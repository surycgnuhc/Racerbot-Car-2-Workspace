"""
bag_query.py

The arithmetic behind the bag_* helper commands: per-topic timing
statistics, finding stretches of time where a condition held, bucketing a
signal down to a readable rate, and shrinking a message to something worth
printing. No rclpy and no mcap -- numbers and plain objects in, plain
dicts out -- so all of it is unit-testable with a bare pytest, the same
split as run_events.py. bag_io.py reads the recording; bag_tools.py is the
command line.

Everything here exists so an answer stays small however big the recording
is: an hour of driving is ~140k LiDAR scans, and the point of these tools
is that nobody -- a person or an AI agent -- has to read them.

Times are nanoseconds in (what the recording stores) and seconds out,
relative to the start of the recording, which is how a person says "what
happened at 1:05".
"""

import math
import struct

from race_diagnostics.field_expr import field_names

NS = 1_000_000_000


def percentile(sorted_values, q):
    """Nearest-rank percentile of an already-sorted list; None if empty."""
    if not sorted_values:
        return None
    rank = max(1, math.ceil(q / 100.0 * len(sorted_values)))
    return sorted_values[rank - 1]


def median(sorted_values):
    if not sorted_values:
        return None
    n = len(sorted_values)
    mid = n // 2
    if n % 2:
        return sorted_values[mid]
    return (sorted_values[mid - 1] + sorted_values[mid]) / 2.0


# --- Header stamps, read straight from the serialized bytes --------------

_HEADER_TYPES = {'Header', 'std_msgs/Header', 'std_msgs/msg/Header'}


def schema_has_header(msgdef_text):
    """True if the message's first field is a std_msgs/Header.

    Reads a ROS 2 .msg definition as stored in the recording. Constants
    (`int32 FOO=1`) are not fields and are skipped. Only the *first* field
    matters: that is the only place header_stamp_ns can find the stamp
    without decoding. (The `===` / `MSG: pkg/Type` lines that introduce
    nested definitions never look like a Header field, so a definition
    with no fields of its own correctly reads as headerless.)
    """
    for raw in (msgdef_text or '').splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2 or '=' in line:
            continue
        return parts[0] in _HEADER_TYPES
    return False


def header_stamp_ns(data):
    """The header stamp of a CDR-serialized message whose first field is a
    std_msgs/Header, without decoding the rest of it.

    Layout, per the OMG CDR encoding ROS 2 uses: 4 bytes of encapsulation
    header (00 00 = big-endian, 00 01 = little-endian), then the Header's
    builtin_interfaces/Time -- int32 sec, uint32 nanosec -- at offsets 4
    and 8. Decoding a whole camera frame to read 8 bytes of it would make
    bag_stats as slow as the frame is big.

    Returns None for anything else: too short, or an encapsulation kind
    this does not recognise.
    """
    if len(data) < 12 or data[0] != 0:
        return None
    if data[1] == 1:
        sec, nanosec = struct.unpack_from('<iI', data, 4)
    elif data[1] == 0:
        sec, nanosec = struct.unpack_from('>iI', data, 4)
    else:
        return None
    return sec * NS + nanosec


# --- Per-topic timing -----------------------------------------------------

class TopicStats:
    """Arrival timing and header-stamp lag for one topic, fed one message
    at a time in arrival order.

    Arrival time is when the recorder received the message. Lag is
    arrival minus the message's own header stamp: how old the data
    already was when it got here. A steady arrival rate says nothing
    about lag -- auto_map_race_node republishes a frozen SLAM pose at a
    perfectly punctual 40 Hz -- which is why both are reported.
    """

    def __init__(self, topic, msg_type=''):
        self.topic = topic
        self.msg_type = msg_type
        self.count = 0
        self.first_ns = None
        self.last_ns = None
        self.periods = []          # (period_ns, arrival_ns of the later message)
        self.lags = []             # (lag_ns, arrival_ns)
        self.last_stamp_ns = None
        self.stamp_backwards = 0

    def add(self, arrival_ns, stamp_ns=None):
        if self.last_ns is not None:
            self.periods.append((arrival_ns - self.last_ns, arrival_ns))
        else:
            self.first_ns = arrival_ns
        self.last_ns = arrival_ns
        self.count += 1
        if stamp_ns is not None:
            self.lags.append((arrival_ns - stamp_ns, arrival_ns))
            if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
                self.stamp_backwards += 1
            self.last_stamp_ns = stamp_ns

    def summary(self, t0_ns, gap_factor=3.0, max_gaps=5):
        """A plain dict, every time in seconds relative to t0_ns.

        A gap is any period longer than gap_factor times the median
        period -- relative to the topic's own rhythm, so a 15 Hz camera
        and a 50 Hz odometry stream are judged by their own standards.
        With fewer than two messages there is no rhythm, so rate, period
        and gaps are None rather than a made-up number.
        """
        def rel(ns):
            return (ns - t0_ns) / NS

        out = {
            'topic': self.topic,
            'type': self.msg_type,
            'count': self.count,
            'start_s': rel(self.first_ns) if self.count else None,
            'end_s': rel(self.last_ns) if self.count else None,
            'rate_hz': None,
            'period_median_s': None,
            'period_p99_s': None,
            'worst_gap_s': None,
            'worst_gap_at_s': None,
            'gap_count': None,
            'gaps': None,
            'lag': None,
        }
        if self.count >= 2:
            span_ns = self.last_ns - self.first_ns
            out['rate_hz'] = (self.count - 1) * NS / span_ns if span_ns > 0 else None
            periods = sorted(p for p, _ in self.periods)
            med = median(periods)
            out['period_median_s'] = med / NS
            out['period_p99_s'] = percentile(periods, 99) / NS
            worst_ns, worst_at = max(self.periods)
            out['worst_gap_s'] = worst_ns / NS
            # A gap is reported at the moment it began: the last message
            # before the silence, which is when someone would go looking.
            out['worst_gap_at_s'] = rel(worst_at - worst_ns)
            limit = gap_factor * med
            gaps = [(p, at) for p, at in self.periods if p > limit]
            out['gap_count'] = len(gaps)
            biggest = sorted(gaps, reverse=True)[:max_gaps]
            out['gaps'] = [{'at_s': rel(at - p), 'length_s': p / NS}
                           for p, at in sorted(biggest, key=lambda g: g[1])]
        if self.lags:
            lags = sorted(lag for lag, _ in self.lags)
            worst_lag, worst_at = max(self.lags)
            out['lag'] = {
                'median_s': median(lags) / NS,
                'p99_s': percentile(lags, 99) / NS,
                'max_s': worst_lag / NS,
                'max_at_s': rel(worst_at),
                'negative_count': sum(1 for lag in lags if lag < 0),
                'stamp_backwards': self.stamp_backwards,
            }
        return out


# --- Finding stretches where a condition held ------------------------------

def find_episodes(samples, min_duration_s=0.0, merge_gap_s=0.0, max_episodes=50):
    """Group consecutive matching samples into episodes.

    samples: iterable of (t_s, matched, report_value) in time order.
        report_value may be None; otherwise each episode reports its min
        and max (finite values only).

    merge_gap_s: two episodes separated by no more than this are one --
        a noisy signal hovering at a threshold otherwise reports dozens
        of fragments of the same event.
    min_duration_s: then drop episodes shorter than this (first matching
        sample to last), which is what removes single-sample electrical
        spikes. An episode of exactly min_duration_s is kept.

    Returns {'episodes': [...], 'total': n, 'shown': m}. Episodes past
    max_episodes are counted but not listed, so the answer stays small.
    """
    raw = []
    current = None
    for t, matched, value in samples:
        if matched:
            if current is None:
                current = {'start_s': t, 'end_s': t, 'samples': 0, 'values': []}
            current['end_s'] = t
            current['samples'] += 1
            if isinstance(value, (int, float)) and math.isfinite(value):
                current['values'].append(value)
        elif current is not None:
            raw.append(current)
            current = None
    if current is not None:
        raw.append(current)

    merged = []
    for ep in raw:
        if merged and ep['start_s'] - merged[-1]['end_s'] <= merge_gap_s:
            last = merged[-1]
            last['end_s'] = ep['end_s']
            last['samples'] += ep['samples']
            last['values'].extend(ep['values'])
        else:
            merged.append(ep)

    kept = []
    for ep in merged:
        duration = ep['end_s'] - ep['start_s']
        if duration < min_duration_s:
            continue
        out = {'start_s': ep['start_s'], 'end_s': ep['end_s'],
               'duration_s': duration, 'samples': ep['samples']}
        if ep['values']:
            out['min'] = min(ep['values'])
            out['max'] = max(ep['values'])
        kept.append(out)
    return {'episodes': kept[:max_episodes], 'total': len(kept),
            'shown': min(len(kept), max_episodes)}


# --- Bucketing a signal down to a readable rate ----------------------------

AGGREGATES = ('mean', 'min', 'max', 'first', 'last')


def _aggregate(values, how):
    if how == 'first':
        return values[0]
    if how == 'last':
        return values[-1]
    finite = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    if not finite:
        return math.nan
    if how == 'mean':
        return sum(finite) / len(finite)
    if how == 'min':
        return min(finite)
    return max(finite)


def downsample(samples, hz, how='mean'):
    """Collapse (t_s, [v1, v2, ...]) samples into one row per 1/hz bucket.

    Buckets are aligned to the start of the recording (bucket k covers
    [k/hz, (k+1)/hz)), so two topics extracted at the same rate line up
    row for row. Each row is (bucket_start_s, [agg(v1), agg(v2), ...]).
    mean/min/max skip NaN and inf; a bucket with nothing finite is nan.
    `min` and `max` are how a short spike survives downsampling -- a mean
    over ten samples shrinks it tenfold.

    hz <= 0 or None means no bucketing: every sample, unchanged.
    """
    if how not in AGGREGATES:
        raise ValueError(f'unknown aggregate "{how}"; use one of {", ".join(AGGREGATES)}')
    if not hz or hz <= 0:
        return [(t, list(vals)) for t, vals in samples]
    rows = []
    bucket = None
    columns = None
    for t, vals in samples:
        k = math.floor(t * hz)
        if k != bucket:
            if bucket is not None:
                rows.append((bucket / hz, [_aggregate(c, how) for c in columns]))
            bucket = k
            columns = [[] for _ in vals]
        for column, v in zip(columns, vals):
            column.append(v)
    if bucket is not None:
        rows.append((bucket / hz, [_aggregate(c, how) for c in columns]))
    return rows


# --- Making a message small enough to print ---------------------------------

def _number_summary(values):
    finite = [v for v in values if math.isfinite(v)]
    out = {'n': len(values)}
    if finite:
        out['min'] = min(finite)
        out['max'] = max(finite)
        out['mean'] = sum(finite) / len(finite)
    nan = sum(1 for v in values if isinstance(v, float) and math.isnan(v))
    inf = sum(1 for v in values if isinstance(v, float) and math.isinf(v))
    if nan:
        out['nan'] = nan
    if inf:
        out['inf'] = inf
    return out


def _plain_number(v):
    if isinstance(v, float) and not math.isfinite(v):
        return str(v)          # 'nan', 'inf', '-inf': JSON has no literal for them
    return v


def compact(value, max_list=8):
    """A JSON-safe, size-bounded version of a decoded message.

    - Any {sec, nanosec} pair becomes float seconds.
    - Lists of more than max_list numbers become a summary: n, min, max,
      mean over finite values, plus nan/inf counts. A 1081-beam scan
      becomes one line instead of 1081 numbers.
    - Other long lists keep their first few elements and their length.
    - bytes become their length (an image's pixels are never useful as text).
    - NaN and inf become strings, because JSON cannot carry them.
    """
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {'bytes': len(value)}
    if isinstance(value, (list, tuple)):
        if len(value) > max_list:
            if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
                return _number_summary([float(v) for v in value])
            return {'n': len(value), 'first': [compact(v, max_list) for v in value[:3]]}
        return [compact(v, max_list) for v in value]
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    if isinstance(value, float):
        return _plain_number(value)
    names = field_names(value)
    if names == ['sec', 'nanosec']:
        return value.sec + value.nanosec / NS
    return {name: compact(getattr(value, name), max_list) for name in names}


def thin_evenly(items, max_items):
    """At most max_items of items, evenly spaced, always keeping the first
    and last. Used where a fixed-size view of a time range is the point
    (bag_window), never where a dropped row could hide the answer
    (bag_extract refuses instead)."""
    n = len(items)
    if max_items <= 0:
        return []
    if n <= max_items:
        return list(items)
    if max_items == 1:
        return [items[0]]
    step = (n - 1) / (max_items - 1)
    return [items[round(i * step)] for i in range(max_items)]


# --- Times typed by a person ------------------------------------------------

def parse_time(text):
    """Seconds from the start of the recording: '65', '65.5', '1:05', '1:05.5'.

    Raises ValueError for anything else, including negative times.
    """
    s = str(text).strip()
    try:
        if ':' in s:
            minutes, seconds = s.split(':', 1)
            if not minutes.isdigit():
                raise ValueError
            seconds = float(seconds)
            if not 0 <= seconds < 60:
                raise ValueError
            value = int(minutes) * 60 + seconds
        else:
            value = float(s)
    except ValueError:
        raise ValueError(f'"{text}" is not a time; use seconds (65) '
                         'or minutes:seconds (1:05)') from None
    if not math.isfinite(value) or value < 0:
        raise ValueError(f'"{text}" is not a time; use seconds (65) or minutes:seconds (1:05)')
    return value


def format_time(seconds):
    """'65.250 s (1:05.25)' -- both forms, since people say both."""
    if seconds is None:
        return '-'
    minutes, rest = divmod(seconds, 60)
    return f'{seconds:.3f} s ({int(minutes)}:{rest:05.2f})'
