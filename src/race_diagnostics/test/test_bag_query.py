"""
Unit tests for bag_query.py: topic timing, header stamps read from raw
bytes, episode finding, downsampling, message compaction and time parsing.

Inputs are hand-built so every expected value is known by construction;
the oracle for each assertion is marked. The same functions are tested
end to end against a noisy fake recording in test_bag_tools.py.

Pure Python, no rclpy, no mcap -- runs unsourced:

    python3 -m pytest src/race_diagnostics/test/test_bag_query.py -v
"""
import math
import os
import struct
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics.bag_query import (TopicStats, compact, downsample,  # noqa: E402
                                        find_episodes, format_time, header_stamp_ns,
                                        median, parse_time, percentile,
                                        schema_has_header, thin_evenly)

S = 1_000_000_000      # ns per second
MS = 1_000_000


# --- percentile / median ----------------------------------------------------

def test_percentile_is_nearest_rank():
    values = list(range(1, 101))                 # 1..100
    assert percentile(values, 99) == 99          # oracle: closed form, ceil(0.99*100) = 99th value
    assert percentile(values, 100) == 100
    assert percentile(values, 50) == 50
    assert percentile([7], 99) == 7              # single element
    assert percentile([], 99) is None


def test_median_of_odd_and_even_lengths():
    assert median([1, 2, 9]) == 2                # oracle: closed form
    assert median([1, 2, 4, 9]) == 3.0           # oracle: closed form, (2+4)/2
    assert median([]) is None


# --- header stamps from serialized bytes -----------------------------------

def cdr(sec, nanosec, little=True, rest=b'\x05\x00\x00\x00odom\x00'):
    # oracle: spec -- OMG CDR: 2-byte representation id (00 01 = CDR_LE,
    # 00 00 = CDR_BE), 2 option bytes, then the Header's Time.
    if little:
        return b'\x00\x01\x00\x00' + struct.pack('<iI', sec, nanosec) + rest
    return b'\x00\x00\x00\x00' + struct.pack('>iI', sec, nanosec) + rest


def test_header_stamp_reads_little_and_big_endian():
    assert header_stamp_ns(cdr(1792411230, 401_854_500)) == 1792411230 * S + 401_854_500
    big_endian = cdr(1792411230, 401_854_500, little=False)
    assert header_stamp_ns(big_endian) == 1792411230 * S + 401_854_500


def test_header_stamp_is_none_for_short_or_unknown_encodings():
    assert header_stamp_ns(cdr(5, 0)[:11]) is None             # one byte short of the stamp
    assert header_stamp_ns(cdr(5, 0)[:12]) == 5 * S            # exactly long enough
    assert header_stamp_ns(b'') is None
    assert header_stamp_ns(b'\x00\x03' + cdr(5, 0)[2:]) is None   # PL_CDR_LE: not plain CDR
    assert header_stamp_ns(b'\x01\x01' + cdr(5, 0)[2:]) is None


def test_header_stamp_handles_negative_seconds():
    # int32 sec is signed in builtin_interfaces/Time
    assert header_stamp_ns(cdr(-1, 500_000_000)) == -S + 500_000_000   # oracle: closed form


@pytest.mark.parametrize('definition, expected', [
    ('std_msgs/Header header\nfloat32[] ranges', True),
    ('Header header\nfloat32[] ranges', True),
    ('std_msgs/msg/Header header', True),
    ('# comment\n\n  std_msgs/Header  header  # trailing\nVescState state', True),
    ('int32 FAULT_CODE_NONE=0\nstd_msgs/Header header', True),     # constants are not fields
    ('string data', False),
    # header not first: stamp not at offset 4
    ('float64 x\nstd_msgs/Header header', False),
    ('string data\n' + '=' * 80 + '\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp', False),
    ('int32 ONLY_A_CONSTANT=1\n' + '=' * 80
     + '\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp', False),
    ('', False),
    (None, False),
])
def test_schema_has_header_only_when_header_is_the_first_field(definition, expected):
    # oracle: spec (CDR layout, see header_stamp_ns)
    assert schema_has_header(definition) is expected


# --- TopicStats ------------------------------------------------------------

def feed(arrivals_ms, stamps_ms=None, t0_ms=0, **kwargs):
    stats = TopicStats('/t', 'pkg/msg/T')
    for i, a in enumerate(arrivals_ms):
        stamp = None if stamps_ms is None else stamps_ms[i] * MS
        stats.add(a * MS, stamp)
    return stats.summary(t0_ms * MS, **kwargs)


def test_rate_and_period_of_a_steady_stream():
    s = feed([1000 + 20 * k for k in range(51)], t0_ms=1000)    # 50 Hz for 1 s
    assert s['count'] == 51
    # oracle: closed form, 50 periods / 1 s
    assert s['rate_hz'] == pytest.approx(50.0, abs=1e-9)
    assert s['period_median_s'] == pytest.approx(0.020, abs=1e-12)  # s; oracle: closed form
    # oracle: closed form
    assert s['start_s'] == 0.0 and s['end_s'] == pytest.approx(1.0, abs=1e-12)
    assert s['gap_count'] == 0 and s['gaps'] == []
    assert s['lag'] is None                                         # no stamps given


def test_a_gap_is_reported_where_the_silence_began():
    arrivals = [20 * k for k in range(10)] + [180 + 500] + [680 + 20 * k for k in range(1, 5)]
    s = feed(arrivals)
    # oracle: closed form, 680 - 180 ms
    assert s['worst_gap_s'] == pytest.approx(0.5, abs=1e-12)
    # oracle: closed form -- last message before the gap
    assert s['worst_gap_at_s'] == pytest.approx(0.18, abs=1e-12)
    assert s['gap_count'] == 1
    assert s['gaps'] == [{'at_s': pytest.approx(0.18, abs=1e-12),  # oracle: closed form
                          'length_s': pytest.approx(0.5, abs=1e-12)}]


def test_gap_threshold_is_strictly_more_than_factor_times_median():
    # median period 10 ms; factor 3 -> 30 ms is not a gap, 31 ms is.
    at_limit = feed([0, 10, 20, 30, 40, 70, 80, 90], gap_factor=3.0)
    past_limit = feed([0, 10, 20, 30, 40, 71, 81, 91], gap_factor=3.0)
    assert at_limit['period_median_s'] == pytest.approx(0.010, abs=1e-12)  # oracle: closed form
    assert at_limit['gap_count'] == 0          # oracle: closed form, 30 > 30 is false
    assert past_limit['gap_count'] == 1        # oracle: closed form, 31 > 30


def test_only_the_biggest_gaps_are_listed_but_all_are_counted_in_time_order():
    arrivals, t = [0], 0
    for big in [100, 300, 200, 400, 500, 600]:
        # three normal periods per gap keep the median at 10 ms
        for length in [big, 10, 10, 10]:
            t += length
            arrivals.append(t)
    s = feed(arrivals, max_gaps=3)
    assert s['period_median_s'] == pytest.approx(0.010, abs=1e-12)  # oracle: closed form
    assert s['gap_count'] == 6                 # oracle: closed form, six periods > 3 x 10 ms
    listed = [g['length_s'] for g in s['gaps']]
    # oracle: closed form -- three biggest, in time order
    assert listed == pytest.approx([0.4, 0.5, 0.6], abs=1e-12)


def test_single_message_and_empty_topics_have_no_rate():
    one = feed([500], t0_ms=0)
    assert one['count'] == 1 and one['start_s'] == 0.5  # oracle: closed form
    assert one['rate_hz'] is None and one['gaps'] is None and one['worst_gap_s'] is None
    empty = TopicStats('/t').summary(0)
    assert empty['count'] == 0 and empty['start_s'] is None and empty['lag'] is None


def test_two_messages_at_the_same_instant_have_no_rate_rather_than_dividing_by_zero():
    s = feed([100, 100])
    assert s['rate_hz'] is None                 # oracle: invariant, zero span
    assert s['worst_gap_s'] == 0.0  # oracle: closed form


def test_lag_is_arrival_minus_stamp_and_reports_when_the_worst_arrived():
    stamps = [0, 20, 40, 60, 80]
    lags = [15, 17, 16, 18, 400]
    s = feed([st + lg for st, lg in zip(stamps, lags)], stamps)
    lag = s['lag']
    assert lag['max_s'] == pytest.approx(0.400, abs=1e-12)          # oracle: closed form
    # oracle: closed form -- stamp 80 + lag 400 ms
    assert lag['max_at_s'] == pytest.approx(0.480, abs=1e-12)
    # oracle: closed form -- sorted lags 15,16,17,18,400
    assert lag['median_s'] == pytest.approx(0.017, abs=1e-12)
    assert lag['p99_s'] == pytest.approx(0.400, abs=1e-12)  # oracle: closed form
    assert lag['negative_count'] == 0 and lag['stamp_backwards'] == 0


def test_a_repeated_stamp_is_not_backwards():
    # A frozen pose republished with its old stamp -- the 2026-07-27 failure
    # (docs/run-diagnostics.md) -- repeats, it does not go backwards; the
    # growing lag is what shows it.
    s = feed([10, 30, 50, 70], stamps_ms=[0, 0, 0, 0])
    assert s['lag']['stamp_backwards'] == 0          # oracle: closed form, 0 < 0 is false
    assert s['lag']['max_s'] == pytest.approx(0.070, abs=1e-12)  # oracle: closed form


def test_future_stamps_and_backwards_stamps_are_counted():
    # stamp 60 arrives at 50 (stamped in the future); then a stamp older than its predecessor
    s = feed([10, 30, 50, 70], stamps_ms=[0, 20, 60, 55])
    assert s['lag']['negative_count'] == 1      # oracle: closed form, 50 - 60 < 0
    assert s['lag']['stamp_backwards'] == 1     # oracle: closed form, 55 < 60


# --- find_episodes ----------------------------------------------------------

def samples(pattern, dt=0.1, values=None):
    """'..##.#' -> (t, matched, value) at dt spacing."""
    return [(i * dt, c == '#', None if values is None else values[i])
            for i, c in enumerate(pattern)]


def test_episodes_are_runs_of_consecutive_matches():
    out = find_episodes(samples('..###..#'))
    eps = out['episodes']
    assert out['total'] == 2
    # oracle: closed form
    assert (eps[0]['start_s'], eps[0]['samples']) == (pytest.approx(0.2, abs=1e-12), 3)
    assert eps[0]['end_s'] == pytest.approx(0.4, abs=1e-12)  # oracle: closed form
    assert eps[0]['duration_s'] == pytest.approx(0.2, abs=1e-12)   # oracle: closed form, 0.4 - 0.2
    # oracle: closed form -- open at the very end, still closed
    assert eps[1]['samples'] == 1 and eps[1]['duration_s'] == 0.0


def test_no_matches_all_matches_and_empty_input():
    assert find_episodes(samples('....'))['total'] == 0
    assert find_episodes([])['episodes'] == []
    full = find_episodes(samples('####'))
    assert full['total'] == 1 and full['episodes'][0]['samples'] == 4


def test_min_duration_drops_spikes_and_keeps_an_episode_of_exactly_that_length():
    pattern = '#..###....#'          # a spike, a 0.2 s episode, a spike
    exact = find_episodes(samples(pattern, dt=0.25), min_duration_s=0.5)
    # oracle: closed form, 3 samples span 0.5 s
    assert exact['total'] == 1
    assert exact['episodes'][0]['start_s'] == pytest.approx(0.75, abs=1e-12)  # oracle: closed form
    longer = find_episodes(samples(pattern, dt=0.25), min_duration_s=0.5 + 1e-9)
    assert longer['total'] == 0                                  # one step past the boundary


def test_merge_gap_joins_flicker_into_one_episode_then_min_duration_applies():
    # a noisy signal hovering at its threshold: matched, missed, matched...
    # dt = 0.25 s keeps every time exact in binary floating point.
    flicker = samples('##.#.##.#', dt=0.25)
    assert find_episodes(flicker)['total'] == 4
    merged = find_episodes(flicker, merge_gap_s=0.5)
    # oracle: closed form, every hole is 0.5 s
    assert merged['total'] == 1
    assert merged['episodes'][0]['samples'] == 6
    assert merged['episodes'][0]['end_s'] == 2.0  # oracle: closed form
    # oracle: closed form -- just short
    assert find_episodes(flicker, merge_gap_s=0.5 - 1e-9)['total'] == 4
    # merging happens before the length filter: alone, no fragment is longer than 0.25 s
    assert find_episodes(flicker, min_duration_s=2.0)['total'] == 0  # oracle: closed form
    # oracle: closed form
    assert find_episodes(flicker, merge_gap_s=0.5, min_duration_s=2.0)['total'] == 1


def test_episode_min_and_max_skip_nonfinite_report_values():
    vals = [0, 10.4, math.nan, 10.1, math.inf, 0]
    ep = find_episodes(samples('.####.', values=vals))['episodes'][0]
    assert (ep['min'], ep['max']) == (10.1, 10.4)       # oracle: closed form over finite values
    no_values = find_episodes(samples('.##.', values=[0, math.nan, math.nan, 0]))['episodes'][0]
    assert 'min' not in no_values and 'max' not in no_values


def test_more_episodes_than_the_cap_are_counted_not_listed():
    out = find_episodes(samples('#.' * 60), max_episodes=50)
    assert out['total'] == 60 and out['shown'] == 50 and len(out['episodes']) == 50
    # oracle: closed form -- the first ones, in time order
    assert out['episodes'][0]['start_s'] == 0.0


# --- downsample -------------------------------------------------------------

def ramp(n=20, hz=10.0):
    """(t, [t*10, -t]) at 10 Hz: values known in closed form."""
    return [(k / hz, [k * 1.0, -k * 1.0]) for k in range(n)]


def test_buckets_align_to_the_recording_start_and_average():
    rows = downsample(ramp(), hz=2.0)
    assert [t for t, _ in rows] == [0.0, 0.5, 1.0, 1.5]        # oracle: closed form, k/2
    # bucket [0, 0.5) holds k = 0..4: mean 2.0; [0.5, 1.0) holds 5..9: mean 7.0
    assert rows[0][1] == pytest.approx([2.0, -2.0], abs=1e-12)  # oracle: closed form
    assert rows[1][1] == pytest.approx([7.0, -7.0], abs=1e-12)  # oracle: closed form


@pytest.mark.parametrize('how, expected', [
    ('min', [0.0, -4.0]), ('max', [4.0, 0.0]), ('first', [0.0, 0.0]), ('last', [4.0, -4.0]),
])
def test_each_aggregate_of_the_first_bucket(how, expected):
    assert downsample(ramp(), hz=2.0, how=how)[0][1] == expected   # oracle: closed form, k = 0..4


def test_min_keeps_a_one_sample_spike_that_mean_dilutes():
    data = [(k / 50, [11.8]) for k in range(50)]
    data[17] = (17 / 50, [10.0])
    assert downsample(data, 1.0, 'min')[0][1] == [10.0]            # oracle: closed form
    mean = downsample(data, 1.0, 'mean')[0][1][0]
    assert mean == pytest.approx(11.8 - 1.8 / 50, abs=1e-9)      # V; oracle: closed form


def test_nonfinite_values_are_skipped_and_an_all_nan_bucket_is_nan():
    data = [(0.0, [math.nan]), (0.1, [2.0]), (0.2, [math.inf]), (0.6, [math.nan])]
    rows = downsample(data, 2.0)
    assert rows[0][1] == [2.0]                 # oracle: closed form over finite values
    assert math.isnan(rows[1][1][0])


def test_bucket_boundary_belongs_to_the_later_bucket():
    rows = downsample([(0.49, [1.0]), (0.5, [5.0])], 2.0)
    # oracle: closed form, [k/hz, (k+1)/hz)
    assert [(t, v) for t, v in rows] == [(0.0, [1.0]), (0.5, [5.0])]


def test_no_hz_passes_every_sample_through_and_empty_input_is_empty():
    data = ramp(3)
    assert downsample(data, None) == data
    assert downsample(data, 0) == data
    assert downsample([], 5.0) == []  # oracle: closed form


def test_unknown_aggregate_is_refused():
    with pytest.raises(ValueError, match='unknown aggregate "median"'):
        downsample(ramp(), 2.0, 'median')


# --- compact -----------------------------------------------------------------

def test_a_long_scan_becomes_a_summary_with_nan_and_inf_counts():
    ranges = [2.0] * 1000 + [0.3, math.nan, math.nan, math.inf]
    out = compact(NS(ranges=ranges))['ranges']
    assert out == {'n': 1004, 'min': 0.3, 'max': 2.0,                    # oracle: closed form
                   'mean': pytest.approx((2000 + 0.3) / 1001, abs=1e-12),
                   'nan': 2, 'inf': 1}


def test_list_length_boundary_and_non_numeric_lists():
    assert compact([1, 2, 3, 4, 5, 6, 7, 8]) == [1, 2, 3, 4, 5, 6, 7, 8]   # exactly max_list: kept
    assert compact(list(range(9)))['n'] == 9                            # one past: summarised
    names = compact(['a'] * 20)
    assert names == {'n': 20, 'first': ['a', 'a', 'a']}
    # bools are not numbers here
    assert compact([True] * 9) == {'n': 9, 'first': [True, True, True]}


def test_time_becomes_seconds_and_nonfinite_becomes_text():
    msg = NS(header=NS(stamp=NS(sec=12, nanosec=500_000_000), frame_id='odom'),
             speed=math.nan, dist=-math.inf, data=b'\x00' * 640, count=3, ok=False)
    assert compact(msg) == {'header': {'stamp': 12.5, 'frame_id': 'odom'},   # oracle: closed form
                            'speed': 'nan', 'dist': '-inf', 'data': {'bytes': 640},
                            'count': 3, 'ok': False}


# --- thin_evenly --------------------------------------------------------------

def test_thin_keeps_first_and_last_and_the_requested_count():
    items = list(range(101))
    assert thin_evenly(items, 5) == [0, 25, 50, 75, 100]   # oracle: closed form, step 25
    assert thin_evenly(items, 1) == [0]
    assert thin_evenly(items, 0) == []
    assert thin_evenly([1, 2], 5) == [1, 2]
    assert thin_evenly([], 3) == []


# --- parse_time / format_time ---------------------------------------------------

@pytest.mark.parametrize('text, seconds', [
    ('65', 65.0), ('65.5', 65.5), ('0', 0.0), ('1:05', 65.0), ('1:05.5', 65.5),
    ('0:59.99', 59.99), ('10:00', 600.0), (' 2:30 ', 150.0),
])
def test_parse_time_accepts_seconds_and_minutes(text, seconds):
    assert parse_time(text) == pytest.approx(seconds, abs=1e-12)   # s; oracle: closed form


@pytest.mark.parametrize('text', ['-1', '1:60', '1:-5', '-1:05', 'abc', '', 'nan', 'inf',
                                  '1:2:3', ':5'])
def test_parse_time_refuses_everything_else(text):
    with pytest.raises(ValueError, match='is not a time'):
        parse_time(text)


def test_format_time_shows_both_forms():
    assert format_time(65.25) == '65.250 s (1:05.25)'     # oracle: closed form
    assert format_time(0.0) == '0.000 s (0:00.00)'  # oracle: closed form
    assert format_time(None) == '-'
