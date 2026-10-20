"""
field_expr.py

A tiny, safe expression language for asking questions about one recorded
message -- `state.voltage_input < 10.5`, `min(ranges) < 0.4`,
`buttons[4] == 1` -- used by bag_extract and bag_find.

No rclpy, no mcap: it walks any object whose fields are attributes (a
decoded ROS message, or a SimpleNamespace in a test), so it is
unit-testable with a bare pytest, the same split as run_events.py.

Expressions are parsed with Python's `ast` and then evaluated by walking
the tree here, never by `eval()`: the text comes from a command line,
possibly typed by an agent, and must not be able to run code. Anything
outside the small whitelist below is refused at compile time with an
ExprError that names the construct.

Allowed:
  field access     header.stamp.sec   state.voltage_input   ranges
  indexing         buttons[4]   axes[-1]          (integer constants only)
  numbers, strings, True/False
  arithmetic       +  -  *  /                    (x / 0 is nan, not an error)
  comparison       <  <=  >  >=  ==  !=          (chains allowed: 0 < x < 1)
  logic            and  or  not
  functions        min max mean abs len nonfinite

min/max/mean ignore NaN and +-inf, because a LaserScan reports a missing
beam as NaN or inf and one of those must not decide the answer. If every
value is non-finite they return nan, and any comparison against nan is
False -- so "no valid beams" never matches `min(ranges) < 0.4`. Ask for
that case directly with `nonfinite(ranges)`, which counts them.
"""

import ast
import math
import operator


class ExprError(ValueError):
    """The expression text is not allowed or does not parse."""


class FieldError(ValueError):
    """The expression names a field the message does not have."""


def _finite(values):
    return [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]


def _as_list(value, name):
    if isinstance(value, (list, tuple)):
        return value
    if isinstance(value, (int, float)):
        return [value]
    raise ExprError(f'{name}() needs numbers or a list of numbers, got {type(value).__name__}')


def _fn_min(*args):
    vals = _finite(v for a in args for v in _as_list(a, 'min'))
    return min(vals) if vals else math.nan


def _fn_max(*args):
    vals = _finite(v for a in args for v in _as_list(a, 'max'))
    return max(vals) if vals else math.nan


def _fn_mean(*args):
    vals = _finite(v for a in args for v in _as_list(a, 'mean'))
    return sum(vals) / len(vals) if vals else math.nan


def _fn_nonfinite(*args):
    return sum(1 for a in args for v in _as_list(a, 'nonfinite')
               if isinstance(v, float) and not math.isfinite(v))


def _fn_abs(value):
    if not isinstance(value, (int, float)):
        raise ExprError(f'abs() needs a number, got {type(value).__name__}')
    return abs(value)


def _fn_len(value):
    if not isinstance(value, (list, tuple, str, bytes)):
        raise ExprError(f'len() needs a list, got {type(value).__name__}')
    return len(value)


FUNCTIONS = {
    'min': _fn_min,
    'max': _fn_max,
    'mean': _fn_mean,
    'abs': _fn_abs,
    'len': _fn_len,
    'nonfinite': _fn_nonfinite,
}

_COMPARE = {
    ast.Lt: operator.lt, ast.LtE: operator.le,
    ast.Gt: operator.gt, ast.GtE: operator.ge,
    ast.Eq: operator.eq, ast.NotEq: operator.ne,
}

_BINARY = {
    ast.Add: operator.add, ast.Sub: operator.sub,
    ast.Mult: operator.mul, ast.Div: operator.truediv,
}


def field_names(obj):
    """The public field names of a decoded message, in declaration order.

    Covers the three shapes a message can arrive in: an rclpy message
    (get_fields_and_field_types), an mcap_ros2 decoded message (a
    SimpleNamespace subclass that lists its fields in __slots__), and a
    plain SimpleNamespace built by a test (__dict__).
    """
    getter = getattr(obj, 'get_fields_and_field_types', None)
    if callable(getter):
        return list(getter().keys())
    slots = getattr(type(obj), '__slots__', None)
    if isinstance(slots, (list, tuple)) and slots:
        return [s for s in slots if not s.startswith('_')]
    if hasattr(obj, '__dict__'):
        return [k for k in vars(obj) if not k.startswith('_')]
    return []


def _get_field(obj, name, path):
    if name.startswith('_'):
        raise ExprError(f'field names starting with "_" are not allowed: {name}')
    names = field_names(obj)
    if name not in names:
        where = path or 'the message'
        raise FieldError(f'{where} has no field "{name}"; '
                         f'available: {", ".join(names) or "(none)"}')
    return getattr(obj, name)


class Expr:
    """A compiled expression. Build with compile_expr()."""

    def __init__(self, text, tree):
        self.text = text
        self._tree = tree

    def evaluate(self, msg):
        try:
            return self._eval(self._tree.body, msg)
        except TypeError as exc:
            # e.g. comparing a whole list to a number: `ranges < 0.4`
            raise ExprError(f'cannot evaluate "{self.text}": {exc}') from None

    def first_operand(self):
        """The left side of a top-level comparison, as its own Expr.

        bag_find reports the min and max of this over each episode, so
        `state.voltage_input < 10.5` says how low the voltage went without
        the user having to ask twice. None if the expression is not a
        plain comparison.
        """
        body = self._tree.body
        if isinstance(body, ast.Compare):
            node = ast.Expression(body=body.left)
            return Expr(ast.unparse(body.left), node)
        return None

    def _eval(self, node, msg):
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Name):
            return _get_field(msg, node.id, '')
        if isinstance(node, ast.Attribute):
            base = self._eval(node.value, msg)
            return _get_field(base, node.attr, ast.unparse(node.value))
        if isinstance(node, ast.Subscript):
            base = self._eval(node.value, msg)
            index = node.slice.value
            if not isinstance(base, (list, tuple)):
                raise ExprError(f'{ast.unparse(node.value)} is not a list')
            if not -len(base) <= index < len(base):
                raise FieldError(f'{ast.unparse(node)} is out of range: '
                                 f'{ast.unparse(node.value)} has {len(base)} elements')
            return base[index]
        if isinstance(node, ast.Call):
            args = [self._eval(a, msg) for a in node.args]
            return FUNCTIONS[node.func.id](*args)
        if isinstance(node, ast.UnaryOp):
            value = self._eval(node.operand, msg)
            if isinstance(node.op, ast.Not):
                return not value
            if isinstance(node.op, ast.USub):
                return -value
            return +value
        if isinstance(node, ast.BinOp):
            left = self._eval(node.left, msg)
            right = self._eval(node.right, msg)
            if isinstance(node.op, ast.Div) and right == 0:
                return math.nan
            return _BINARY[type(node.op)](left, right)
        if isinstance(node, ast.BoolOp):
            if isinstance(node.op, ast.And):
                return all(self._eval(v, msg) for v in node.values)
            return any(self._eval(v, msg) for v in node.values)
        if isinstance(node, ast.Compare):
            left = self._eval(node.left, msg)
            for op, comparator in zip(node.ops, node.comparators):
                right = self._eval(comparator, msg)
                if not _COMPARE[type(op)](left, right):
                    return False
                left = right
            return True
        raise ExprError(f'unsupported expression: {ast.unparse(node)}')  # unreachable after _check


def _check(node):
    """Refuse every node type outside the whitelist, before anything runs."""
    if isinstance(node, ast.Expression):
        _check(node.body)
    elif isinstance(node, ast.Constant):
        if not isinstance(node.value, (int, float, str, bool)):
            raise ExprError(f'unsupported constant: {node.value!r}')
    elif isinstance(node, ast.Name):
        if node.id.startswith('_'):
            raise ExprError(f'field names starting with "_" are not allowed: {node.id}')
    elif isinstance(node, ast.Attribute):
        if node.attr.startswith('_'):
            raise ExprError(f'field names starting with "_" are not allowed: {node.attr}')
        if not isinstance(node.value, (ast.Name, ast.Attribute, ast.Subscript)):
            raise ExprError(f'can only read fields of a message: {ast.unparse(node)}')
        _check(node.value)
    elif isinstance(node, ast.Subscript):
        index = node.slice
        if not (isinstance(index, ast.Constant) and type(index.value) is int):
            if (isinstance(index, ast.UnaryOp) and isinstance(index.op, ast.USub)
                    and isinstance(index.operand, ast.Constant)
                    and type(index.operand.value) is int):
                node.slice = ast.Constant(-index.operand.value)
            else:
                raise ExprError(f'index must be a whole number: {ast.unparse(node)}')
        if not isinstance(node.value, (ast.Name, ast.Attribute, ast.Subscript)):
            raise ExprError(f'can only index a field: {ast.unparse(node)}')
        _check(node.value)
    elif isinstance(node, ast.Call):
        if not isinstance(node.func, ast.Name) or node.func.id not in FUNCTIONS:
            raise ExprError(f'unknown function in {ast.unparse(node)}; '
                            f'allowed: {", ".join(sorted(FUNCTIONS))}')
        if node.keywords:
            raise ExprError(f'keyword arguments are not allowed: {ast.unparse(node)}')
        if not node.args:
            raise ExprError(f'{node.func.id}() needs an argument')
        for arg in node.args:
            if isinstance(arg, ast.Starred):
                raise ExprError(f'*args is not allowed: {ast.unparse(node)}')
            _check(arg)
    elif isinstance(node, ast.UnaryOp):
        if not isinstance(node.op, (ast.Not, ast.USub, ast.UAdd)):
            raise ExprError(f'unsupported operator in {ast.unparse(node)}')
        _check(node.operand)
    elif isinstance(node, ast.BinOp):
        if type(node.op) not in _BINARY:
            raise ExprError(f'unsupported operator in {ast.unparse(node)}; '
                            f'allowed: + - * /')
        _check(node.left)
        _check(node.right)
    elif isinstance(node, ast.BoolOp):
        for value in node.values:
            _check(value)
    elif isinstance(node, ast.Compare):
        for op in node.ops:
            if type(op) not in _COMPARE:
                raise ExprError(f'unsupported comparison in {ast.unparse(node)}; '
                                f'allowed: < <= > >= == !=')
        _check(node.left)
        for comparator in node.comparators:
            _check(comparator)
    else:
        raise ExprError(f'not allowed in an expression: {type(node).__name__} '
                        f'({ast.unparse(node)})')


def compile_expr(text):
    """Parse and vet `text`. Raises ExprError if it is not allowed."""
    if not isinstance(text, str) or not text.strip():
        raise ExprError('empty expression')
    try:
        tree = ast.parse(text.strip(), mode='eval')
    except SyntaxError as exc:
        raise ExprError(f'cannot parse "{text}": {exc.msg}') from None
    _check(tree)
    return Expr(text.strip(), tree)
