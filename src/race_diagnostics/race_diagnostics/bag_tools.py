"""
bag_tools.py

Four small commands that answer questions about a recording made by
record_run.py, each printing a short answer instead of the data:

    bag_stats   RUN                         what is in it: each topic's rate,
                                            gaps, and how old its data was
    bag_window  RUN --at 1:05               a few seconds of every topic,
                                            long arrays summarised
    bag_extract RUN TOPIC EXPR...           one or more values over time as
                --hz 5                      CSV, bucketed to a readable rate
    bag_find    RUN TOPIC CONDITION         when a condition held, as a list
                                            of episodes

    ros2 run race_diagnostics bag_stats ~/.ros/racerbot_runs/20261019-140211
    ros2 run race_diagnostics bag_find <run> /sensors/core \\
        "state.voltage_input < 10.5" --min-duration 0.1

They exist so that a person or an AI agent can investigate an hour-long
recording without anyone reading it: the answer stays the same size
however big the recording gets. Times on the command line and in the
output are seconds from the start of the recording, or minutes:seconds.

Read-only: opens the recording's files and nothing else, and is not a ROS
node. Expression syntax is in field_expr.py; the arithmetic in
bag_query.py; file reading in bag_io.py. Full guide:
docs/run-diagnostics.md#asking-questions-about-a-recording
"""

import argparse
import csv
import json
import math
import sys
import time
from collections import defaultdict

from race_diagnostics.bag_io import Bag, BagError
from race_diagnostics.bag_query import (AGGREGATES, TopicStats, compact, downsample,
                                        find_episodes, format_time, header_stamp_ns,
                                        parse_time, thin_evenly)
from race_diagnostics.field_expr import ExprError, FieldError, compile_expr

# docs/run-diagnostics.md's pose-lag table: under 0.15 s is healthy.
LAG_WORTH_A_LOOK_S = 0.15


def _time_arg(text):
    try:
        return parse_time(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from None


def _parser(name, doc):
    parser = argparse.ArgumentParser(
        prog=name, description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recording', help='run directory, its bag/ directory, or one .mcap file')
    return parser


def _run(fn, argv, parser):
    args = parser.parse_args(argv)
    try:
        return fn(args)
    except (BagError, ExprError, FieldError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


def _header(bag):
    started = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(bag.start_ns / 1e9))
    files = f'{len(bag.files)} file' + ('s' if len(bag.files) != 1 else '')
    return (f'Recording: {bag.files[0].parent}  '
            f'({files}, {bag.duration_s:.3f} s, started {started})')


def _ms(seconds):
    return '-' if seconds is None else f'{seconds * 1000:.1f} ms'


def _plain(value):
    """A JSON-safe version of a result dict: nan/inf become strings."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_plain(v) for v in value]
    return value


# --- bag_stats ----------------------------------------------------------

def stats(bag, topics=None, gap_factor=3.0):
    """Per-topic timing for the whole recording (or just `topics`)."""
    if topics:
        bag.check_topics(topics)
    wanted = topics or sorted(bag.topics)
    per_topic = {t: TopicStats(t, bag.topics[t]['type']) for t in wanted}
    has_header = {t: bag.topics[t]['header'] for t in wanted}
    for topic, arrival_ns, data in bag.raw(topics=wanted):
        stamp = header_stamp_ns(data) if has_header[topic] else None
        per_topic[topic].add(arrival_ns, stamp)
    return {
        'recording': str(bag.files[0].parent),
        'files': [str(f) for f in bag.files],
        'duration_s': bag.duration_s,
        'gap_factor': gap_factor,
        'topics': [per_topic[t].summary(bag.start_ns, gap_factor) for t in wanted],
    }


def worth_a_look(result):
    """The lines a reader should not miss, pulled out of the per-topic detail."""
    notes = []
    for s in result['topics']:
        if s['count'] == 0:
            notes.append(f'{s["topic"]}: no messages at all')
            continue
        if s['gap_count']:
            notes.append(f'{s["topic"]}: {s["gap_count"]} gap(s) over {result["gap_factor"]:g}x '
                         f'its usual period; worst {_ms(s["worst_gap_s"])} '
                         f'at {format_time(s["worst_gap_at_s"])}')
        lag = s['lag']
        if lag and lag['max_s'] > LAG_WORTH_A_LOOK_S:
            notes.append(f'{s["topic"]}: data up to {_ms(lag["max_s"])} old on arrival, '
                         f'at {format_time(lag["max_at_s"])}')
        if lag and lag['negative_count']:
            notes.append(f'{s["topic"]}: {lag["negative_count"]} message(s) stamped in the '
                         'future -- two clocks disagree')
        if lag and lag['stamp_backwards']:
            notes.append(f'{s["topic"]}: header stamp went backwards '
                         f'{lag["stamp_backwards"]} time(s)')
    return notes


def render_stats(bag, result):
    lines = [_header(bag), '']
    notes = worth_a_look(result)
    lines.append('Worth a look:' if notes else 'Worth a look: nothing -- no gaps, no stale data.')
    lines += [f'  - {n}' for n in notes]
    lines.append('')
    for s in result['topics']:
        lines.append(f'{s["topic"]}  {s["type"]}')
        if s['rate_hz'] is None:
            lines.append(f'  {s["count"]} msg(s), at {format_time(s["start_s"])}')
        else:
            lines.append(f'  {s["count"]} msgs, {s["rate_hz"]:.1f} Hz '
                         f'(median period {_ms(s["period_median_s"])}, '
                         f'p99 {_ms(s["period_p99_s"])}), '
                         f'{format_time(s["start_s"])} to {format_time(s["end_s"])}')
            lines.append(f'  worst gap {_ms(s["worst_gap_s"])} '
                         f'at {format_time(s["worst_gap_at_s"])}')
        lag = s['lag']
        if lag:
            lines.append(f'  lag (arrival - header stamp): median {_ms(lag["median_s"])}, '
                         f'p99 {_ms(lag["p99_s"])}, max {_ms(lag["max_s"])} '
                         f'at {format_time(lag["max_at_s"])}')
    return '\n'.join(lines)


def stats_main(argv=None):
    parser = _parser('bag_stats', 'What is in a recording: per-topic rate, gaps and lag.')
    parser.add_argument('--topics', nargs='+', help='only these topics (default: all)')
    parser.add_argument('--gap-factor', type=float, default=3.0,
                        help='a gap is a period longer than this times the median (default 3)')
    parser.add_argument('--json', action='store_true', help='machine-readable output')

    def go(args):
        if args.gap_factor <= 1:
            raise ValueError('--gap-factor must be greater than 1')
        bag = Bag(args.recording)
        result = stats(bag, args.topics, args.gap_factor)
        if args.json:
            result['worth_a_look'] = worth_a_look(result)
            print(json.dumps(_plain(result), indent=1))
        else:
            print(render_stats(bag, result))
        return 0
    return _run(go, argv, parser)


# --- bag_window ---------------------------------------------------------

def window(bag, at_s, span_s, topics=None, max_rows=5):
    """Every topic's messages within span_s/2 of at_s, compacted, at most
    max_rows per topic (evenly spaced, first and last always kept)."""
    if topics:
        bag.check_topics(topics)
    start = max(0.0, at_s - span_s / 2)
    end = at_s + span_s / 2
    rows = defaultdict(list)
    for topic, arrival_ns, msg in bag.decoded(topics=topics, start_s=start, end_s=end):
        rows[topic].append(((arrival_ns - bag.start_ns) / 1e9, msg))
    out = {'at_s': at_s, 'start_s': start, 'end_s': end, 'topics': []}
    for topic in sorted(topics or bag.topics):
        found = rows.get(topic, [])
        shown = thin_evenly(found, max_rows)
        out['topics'].append({
            'topic': topic, 'in_window': len(found), 'shown': len(shown),
            'messages': [{'t_s': t, 'msg': compact(m)} for t, m in shown],
        })
    return out


def render_window(bag, result):
    lines = [_header(bag),
             f'Window {format_time(result["start_s"])} to {format_time(result["end_s"])}', '']
    for t in result['topics']:
        lines.append(f'{t["topic"]}  ({t["in_window"]} msgs in window, showing {t["shown"]})')
        for m in t['messages']:
            lines.append(f'  {m["t_s"]:.3f} s  {json.dumps(m["msg"], separators=(",", ":"))}')
    return '\n'.join(lines)


def window_main(argv=None):
    parser = _parser('bag_window', 'A few seconds of a recording, every topic, summarised.')
    parser.add_argument('--at', type=_time_arg, required=True,
                        help='centre of the window: seconds (65) or minutes:seconds (1:05)')
    parser.add_argument('--span', type=float, default=2.0, help='window length, s (default 2)')
    parser.add_argument('--topics', nargs='+', help='only these topics (default: all)')
    parser.add_argument('--max-rows', type=int, default=5,
                        help='messages shown per topic, evenly spaced (default 5)')
    parser.add_argument('--json', action='store_true', help='machine-readable output')

    def go(args):
        if not args.span > 0:
            raise ValueError('--span must be positive')
        if args.max_rows < 1:
            raise ValueError('--max-rows must be at least 1')
        bag = Bag(args.recording)
        result = window(bag, args.at, args.span, args.topics, args.max_rows)
        print(json.dumps(_plain(result), indent=1) if args.json else render_window(bag, result))
        return 0
    return _run(go, argv, parser)


# --- bag_extract --------------------------------------------------------

def _number(value, expr):
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, (list, tuple)):
        raise ExprError(f'"{expr.text}" is a list of {len(value)} values; reduce it to one '
                        'number with min(), max(), mean() or len()')
    raise ExprError(f'"{expr.text}" is not a number (it is {type(value).__name__}); '
                    'name a numeric field inside it')


def extract(bag, topic, expressions, hz=None, how='mean', start_s=None, end_s=None):
    """Rows of (t_s, [value per expression]), bucketed to hz if given."""
    bag.check_topics([topic])
    exprs = [compile_expr(e) for e in expressions]

    def samples():
        for _topic, arrival_ns, msg in bag.decoded(topics=[topic], start_s=start_s, end_s=end_s):
            yield ((arrival_ns - bag.start_ns) / 1e9,
                   [_number(e.evaluate(msg), e) for e in exprs])

    return downsample(samples(), hz, how)


def _cell(value):
    return f'{value:.9g}' if math.isfinite(value) else str(value)


def extract_main(argv=None):
    parser = _parser('bag_extract', 'Values from one topic over time, as CSV.')
    parser.add_argument('topic')
    parser.add_argument('expressions', nargs='+', metavar='EXPR',
                        help='one column each, e.g. state.voltage_input "min(ranges)"')
    parser.add_argument('--hz', type=float, default=None,
                        help='one row per 1/hz seconds (default: every message)')
    parser.add_argument('--agg', choices=AGGREGATES, default='mean',
                        help='how a bucket becomes one row (default mean; min/max keep spikes)')
    parser.add_argument('--start', type=_time_arg, default=None, help='from this time')
    parser.add_argument('--end', type=_time_arg, default=None, help='up to this time')
    parser.add_argument('--max-rows', type=int, default=2000,
                        help='refuse to print more rows than this '
                             '(default 2000; no limit with -o)')
    parser.add_argument('-o', '--output', help='write the CSV here instead of printing it')

    def go(args):
        if args.hz is not None and not args.hz > 0:
            raise ValueError('--hz must be positive')
        bag = Bag(args.recording)
        rows = extract(bag, args.topic, args.expressions, args.hz, args.agg, args.start, args.end)
        if not args.output and len(rows) > args.max_rows:
            # Refuse rather than thin: a dropped row could be the one that matters.
            raise ValueError(f'{len(rows)} rows is more than --max-rows {args.max_rows}. '
                             'Lower --hz, narrow --start/--end, or write to a file with -o.')
        stream = open(args.output, 'w', newline='') if args.output else sys.stdout
        try:
            writer = csv.writer(stream)
            writer.writerow(['t_s'] + args.expressions)
            for t, values in rows:
                writer.writerow([f'{t:.3f}'] + [_cell(v) for v in values])
        finally:
            if args.output:
                stream.close()
        if args.output:
            print(f'wrote {len(rows)} rows to {args.output}')
        return 0
    return _run(go, argv, parser)


# --- bag_find -----------------------------------------------------------

def find(bag, topic, condition, min_duration_s=0.0, merge_gap_s=0.0, report=None,
         start_s=None, end_s=None, max_episodes=50):
    """Episodes where `condition` held on `topic`'s messages."""
    bag.check_topics([topic])
    cond = compile_expr(condition)
    reported = compile_expr(report) if report else cond.first_operand()

    def samples():
        for _topic, arrival_ns, msg in bag.decoded(topics=[topic], start_s=start_s, end_s=end_s):
            value = reported.evaluate(msg) if reported else None
            yield (arrival_ns - bag.start_ns) / 1e9, bool(cond.evaluate(msg)), value

    result = find_episodes(samples(), min_duration_s, merge_gap_s, max_episodes)
    result.update({'topic': topic, 'condition': cond.text,
                   'reported': reported.text if reported else None,
                   'min_duration_s': min_duration_s, 'merge_gap_s': merge_gap_s})
    return result


def render_find(bag, result):
    lines = [_header(bag),
             f'{result["topic"]}: {result["condition"]}  '
             f'(min-duration {result["min_duration_s"]:g} s, '
             f'merge-gap {result["merge_gap_s"]:g} s)',
             '']
    if not result['total']:
        lines.append('Never true.')
    else:
        more = ''
        if result['shown'] < result['total']:
            more = f', showing the first {result["shown"]}'
        lines.append(f'{result["total"]} episode(s){more}:')
    for ep in result['episodes']:
        line = (f'  {format_time(ep["start_s"])} -> {format_time(ep["end_s"])}  '
                f'lasted {ep["duration_s"]:.3f} s, {ep["samples"]} msgs')
        if 'min' in ep:
            line += f', {result["reported"]} {ep["min"]:.6g} .. {ep["max"]:.6g}'
        lines.append(line)
    return '\n'.join(lines)


def find_main(argv=None):
    parser = _parser('bag_find', 'When a condition held on one topic, as episodes.')
    parser.add_argument('topic')
    parser.add_argument('condition',
                        help='e.g. "state.voltage_input < 10.5" or "min(ranges) < 0.4"')
    parser.add_argument('--min-duration', type=float, default=0.0,
                        help='ignore episodes shorter than this, s '
                             '-- filters noise spikes (default 0)')
    parser.add_argument('--merge-gap', type=float, default=0.0,
                        help='join episodes closer together than this, s (default 0)')
    parser.add_argument('--report', help='also report the min/max of this expression per episode '
                        '(default: the left side of the condition)')
    parser.add_argument('--start', type=_time_arg, default=None, help='from this time')
    parser.add_argument('--end', type=_time_arg, default=None, help='up to this time')
    parser.add_argument('--max-episodes', type=int, default=50,
                        help='episodes listed (default 50)')
    parser.add_argument('--json', action='store_true', help='machine-readable output')

    def go(args):
        if args.min_duration < 0 or args.merge_gap < 0:
            raise ValueError('--min-duration and --merge-gap cannot be negative')
        if args.max_episodes < 1:
            raise ValueError('--max-episodes must be at least 1')
        bag = Bag(args.recording)
        result = find(bag, args.topic, args.condition, args.min_duration, args.merge_gap,
                      args.report, args.start, args.end, args.max_episodes)
        print(json.dumps(_plain(result), indent=1) if args.json else render_find(bag, result))
        return 0
    return _run(go, argv, parser)
