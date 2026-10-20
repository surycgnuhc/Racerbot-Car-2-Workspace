"""
Unit tests for field_expr.py, the expression language bag_extract and
bag_find evaluate against each recorded message.

Messages here are SimpleNamespaces shaped like the real ones (field names
from the .msg files cited in fake_recording.py); every expected value is
recomputed in the test from those fields, so the oracle is closed form
throughout unless marked otherwise.

Pure Python, no rclpy, no mcap -- runs unsourced:

    python3 -m pytest src/race_diagnostics/test/test_field_expr.py -v
"""
import math
import os
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics.field_expr import (ExprError, FieldError,  # noqa: E402
                                         compile_expr, field_names)


def vesc(volts=11.8, current=8.0):
    return NS(header=NS(stamp=NS(sec=5, nanosec=250_000_000), frame_id='vesc'),
              state=NS(voltage_input=volts, current_motor=current))


def scan(ranges):
    return NS(header=NS(stamp=NS(sec=1, nanosec=0), frame_id='laser'), ranges=list(ranges))


def joy(buttons):
    return NS(axes=[0.0] * 8, buttons=list(buttons))


def ev(text, msg):
    return compile_expr(text).evaluate(msg)


# --- reading fields --------------------------------------------------------

def test_nested_field_reads_the_value_at_that_path():
    msg = vesc(volts=10.37)
    # oracle: closed form (the field set above)
    assert ev('state.voltage_input', msg) == 10.37
    assert ev('header.stamp.nanosec', msg) == 250_000_000   # oracle: closed form


def test_index_reads_one_element_and_negative_counts_from_the_end():
    msg = joy([0, 0, 0, 0, 1, 0, 7])
    assert ev('buttons[4]', msg) == 1      # oracle: closed form -- LB is index 4
    assert ev('buttons[-1]', msg) == 7     # oracle: closed form
    assert ev('buttons[0]', msg) == 0      # oracle: closed form


def test_index_one_past_the_end_is_a_field_error_not_an_index_error():
    msg = joy([0, 1])
    assert ev('buttons[1]', msg) == 1      # oracle: closed form -- last valid index
    with pytest.raises(FieldError, match=r'buttons\[2\] is out of range: buttons has 2 elements'):
        ev('buttons[2]', msg)
    with pytest.raises(FieldError, match='out of range'):
        ev('buttons[-3]', msg)


def test_unknown_field_names_the_fields_that_do_exist():
    with pytest.raises(FieldError, match=r'state has no field "voltage"; available: '
                                         r'voltage_input, current_motor'):
        ev('state.voltage', vesc())
    with pytest.raises(FieldError, match=r'the message has no field "speed"'):
        ev('speed', vesc())


def test_field_names_reads_each_message_shape():
    class Slotted(NS):           # how mcap_ros2 builds decoded messages
        __slots__ = ['header', '_cache', 'ranges']

    class RclpyLike:             # rclpy messages expose this classmethod
        @staticmethod
        def get_fields_and_field_types():
            return {'axes': 'sequence<float>', 'buttons': 'sequence<int32>'}

    # oracle: invariant (declared order)
    assert field_names(Slotted()) == ['header', 'ranges']
    assert field_names(RclpyLike()) == ['axes', 'buttons']      # oracle: invariant
    assert field_names(NS(b=1, a=2, _hidden=3)) == ['b', 'a']   # oracle: invariant
    assert field_names(3.0) == []                               # oracle: invariant


# --- arithmetic, comparison, logic -----------------------------------------

def test_arithmetic_follows_operator_precedence():
    msg = vesc(volts=12.0, current=3.0)
    # oracle: closed form, 12 * 3 + 12 / 3 - 1 = 39
    expr = ('state.voltage_input * state.current_motor'
            ' + state.voltage_input / state.current_motor - 1')
    assert ev(expr, msg) == pytest.approx(39.0, abs=1e-12)   # dimensionless
    assert ev('-state.current_motor', msg) == -3.0     # oracle: closed form (sign)


def test_division_by_zero_is_nan_so_one_bad_message_does_not_stop_a_scan():
    value = ev('state.voltage_input / state.current_motor', vesc(current=0.0))
    assert math.isnan(value)   # oracle: spec (field_expr.py docstring: x / 0 is nan)


def test_comparison_is_exact_at_the_boundary():
    at = vesc(volts=10.5)
    assert ev('state.voltage_input < 10.5', at) is False    # oracle: closed form
    assert ev('state.voltage_input <= 10.5', at) is True    # oracle: closed form
    assert ev('state.voltage_input < 10.5', vesc(volts=10.499)) is True  # oracle: closed form
    assert ev('state.voltage_input > 10.5', vesc(volts=10.501)) is True  # oracle: closed form
    assert ev('state.voltage_input != 10.5', at) is False  # oracle: closed form


def test_chained_comparison_needs_every_link():
    assert ev('10 < state.voltage_input < 12', vesc(volts=11.0)) is True    # oracle: closed form
    # oracle: closed form -- right link fails
    assert ev('10 < state.voltage_input < 12', vesc(volts=12.0)) is False
    # oracle: closed form -- left link fails
    assert ev('10 < state.voltage_input < 12', vesc(volts=10.0)) is False


def test_and_or_not_combine_conditions():
    msg = vesc(volts=10.0, current=20.0)
    # oracle: closed form
    assert ev('state.voltage_input < 10.5 and state.current_motor > 15', msg) is True
    # oracle: closed form
    assert ev('state.voltage_input < 10.5 and state.current_motor > 25', msg) is False
    assert ev('state.voltage_input > 11 or state.current_motor > 15', msg) is True
    assert ev('state.voltage_input > 11 or state.current_motor > 25', msg) is False
    assert ev('not state.voltage_input > 11', msg) is True


def test_a_string_field_compares_by_equality():
    assert ev("header.frame_id == 'vesc'", vesc()) is True     # oracle: closed form
    assert ev("header.frame_id == 'laser'", vesc()) is False


def test_comparing_a_whole_list_to_a_number_is_an_expr_error():
    with pytest.raises(ExprError, match=r'cannot evaluate "ranges < 0.4"'):
        ev('ranges < 0.4', scan([1.0, 2.0]))


# --- functions over LiDAR-shaped data --------------------------------------

def test_min_max_mean_ignore_nan_and_inf_beams():
    ranges = [math.nan, 2.0, math.inf, 0.5, -math.inf, 3.5, math.nan]
    msg = scan(ranges)
    finite = [2.0, 0.5, 3.5]
    assert ev('min(ranges)', msg) == min(finite)                      # oracle: closed form
    assert ev('max(ranges)', msg) == max(finite)                      # oracle: closed form
    assert ev('mean(ranges)', msg) == pytest.approx(sum(finite) / 3, abs=1e-12)   # m


def test_a_leading_nan_beam_does_not_hide_a_close_obstacle():
    # Python's own min([nan, 0.3]) is nan, and nan < 0.4 is False: a scan
    # whose first beam is missing would hide a wall at 0.3 m.
    msg = scan([math.nan, 3.0, 0.3, 2.0])
    assert ev('min(ranges) < 0.4', msg) is True     # oracle: closed form


def test_all_nonfinite_scan_never_matches_a_threshold_but_can_be_counted():
    msg = scan([math.nan, math.inf, math.nan])
    assert math.isnan(ev('min(ranges)', msg))     # oracle: spec (field_expr.py docstring)
    assert ev('min(ranges) < 0.4', msg) is False  # oracle: spec field_expr.py docstring
    assert ev('min(ranges) >= 0.4', msg) is False  # oracle: spec field_expr.py docstring
    assert ev('nonfinite(ranges)', msg) == 3      # oracle: closed form


def test_empty_and_single_element_lists():
    assert math.isnan(ev('min(ranges)', scan([])))          # oracle: invariant (nothing finite)
    assert ev('len(ranges)', scan([])) == 0
    assert ev('nonfinite(ranges)', scan([])) == 0
    assert ev('max(ranges)', scan([1.25])) == 1.25          # oracle: closed form
    assert ev('mean(ranges)', scan([1.25])) == 1.25  # oracle: closed form


def test_len_and_nonfinite_count_beams():
    msg = scan([1.0, math.nan, 2.0, math.inf, math.inf])
    assert ev('len(ranges)', msg) == 5            # oracle: closed form
    assert ev('nonfinite(ranges)', msg) == 3      # oracle: closed form


def test_min_of_several_scalars_and_abs():
    msg = vesc(volts=11.0, current=-4.0)
    assert ev('min(state.voltage_input, state.current_motor)', msg) == -4.0   # oracle: closed form
    assert ev('abs(state.current_motor)', msg) == 4.0                         # oracle: closed form
    with pytest.raises(ExprError, match=r'abs\(\) needs a number, got list'):
        ev('abs(ranges)', scan([1.0]))
    with pytest.raises(ExprError, match=r'len\(\) needs a list, got float'):
        ev('len(state.voltage_input)', msg)


# --- refusals: the text comes from a command line and must not run code -----

@pytest.mark.parametrize('text, message', [
    ("__import__('os').system('true')", 'unknown function'),
    ('open("/etc/passwd")', 'unknown function'),
    ('header.__class__', 'starting with "_"'),
    ('_private', 'starting with "_"'),
    ('(lambda: 1)()', 'unknown function'),
    ('[x for x in ranges]', 'not allowed in an expression: ListComp'),
    ('min(ranges, key=abs)', 'keyword arguments are not allowed'),
    ('min(*ranges)', r'\*args is not allowed'),
    ('min()', r'min\(\) needs an argument'),
    ('ranges[0:3]', 'index must be a whole number'),
    ('ranges[i]', 'index must be a whole number'),
    ('ranges[1.5]', 'index must be a whole number'),
    ('min(ranges).real', 'can only read fields of a message'),
    ('state.voltage_input ** 2', r'allowed: \+ - \* /'),
    ('state.voltage_input in ranges', r'allowed: < <= > >= == !='),
    ('x if y else z', 'not allowed in an expression: IfExp'),
    ('None', 'unsupported constant'),
    ('state.voltage_input <', 'cannot parse'),
    ('', 'empty expression'),
    ('   ', 'empty expression'),
])
def test_refused_at_compile_time(text, message):
    with pytest.raises(ExprError, match=message):
        compile_expr(text)


def test_first_operand_is_the_left_side_of_a_comparison_only():
    # oracle: closed form
    assert compile_expr('state.voltage_input < 10.5').first_operand().text == 'state.voltage_input'
    # oracle: closed form
    assert compile_expr('min(ranges) <= 0.4').first_operand().evaluate(scan([0.3, 1.0])) == 0.3
    assert compile_expr('a < 1 and b > 2').first_operand() is None
    assert compile_expr('state.voltage_input').first_operand() is None
