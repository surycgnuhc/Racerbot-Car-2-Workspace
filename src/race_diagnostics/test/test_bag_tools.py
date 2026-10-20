"""
End-to-end tests for the bag_* commands (bag_tools.py + bag_io.py) against
a noisy fake recording written by fake_recording.py.

The oracle throughout is the generator's FakeTruth: the events it injected
(when, how long, how big) and the bounds it put on its own jitter. Every
tolerance below is derived from those bounds, never from what the tools
happened to print -- e.g. a dropout of D seconds at period P must measure
D + P, give or take two stamp jitters plus the spread of the latency.

The fake recording is deliberately messy (see fake_recording.py): jittered
stamps, variable latency, Gaussian noise, NaN/inf beams, and one-sample
glitches that a naive threshold search reports as events. Several tests
check that noise *is* seen without a filter, so a test cannot pass merely
because the noise was too small to matter.

Needs the mcap libraries (bag_io.py's INSTALL_HINT); without them this file
fails to collect, loudly, rather than skipping:

    python3 -m pytest src/race_diagnostics/test/test_bag_tools.py -v
"""
import csv
import io
import json
import math
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
from race_diagnostics.bag_io import INSTALL_HINT, Bag, BagError  # noqa: E402

try:
    from fake_recording import write_fake_run
except ImportError as exc:        # say how to fix it, still as a loud collection error
    raise ImportError(f'{exc}. These tests need the mcap libraries: {INSTALL_HINT}') from None

from race_diagnostics import bag_tools  # noqa: E402

US = 1e-6      # 1 microsecond: arrival times are rounded to whole nanoseconds


@pytest.fixture(scope='module')
def run(tmp_path_factory):
    run_dir = tmp_path_factory.mktemp('runs') / '20261019-120000'
    truth = write_fake_run(run_dir, seed=7)
    return run_dir, truth


def spread(truth, topic):
    """How far one arrival can stray from nominal: stamp jitter + latency."""
    lo, hi = truth.latency_s[topic]
    return truth.stamp_jitter_s, lo, hi


def by_topic(result):
    return {s['topic']: s for s in result['topics']}


# --- reading the recording ------------------------------------------------

def test_both_split_files_are_read_and_every_message_counted(run):
    run_dir, truth = run
    bag = Bag(run_dir)
    assert [f.name for f in bag.files] == ['bag_0.mcap', 'bag_1.mcap']
    stats = by_topic(bag_tools.stats(bag))
    for topic, count in truth.counts.items():
        assert stats[topic]['count'] == count      # oracle: the generator's own tally
        assert bag.topics[topic]['count'] == count  # summary section agrees with a full read
    assert bag.start_ns == truth.start_ns           # /robot_description arrives at exactly t = 0


def test_the_bag_directory_and_a_single_file_open_too(run):
    run_dir, truth = run
    assert Bag(run_dir / 'bag').files == Bag(run_dir).files
    single = Bag(run_dir / 'bag' / 'bag_1.mcap')
    assert single.start_ns >= truth.start_ns + int(truth.split_at_s * 1e9)   # second half only


def test_header_detection_matches_the_message_definitions(run):
    bag = Bag(run[0])
    # oracle: spec -- every type here leads with std_msgs/Header except String
    assert {t: v['header'] for t, v in bag.topics.items()} == {
        '/robot_description': False, '/odom': True, '/drive': True,
        '/scan': True, '/sensors/core': True, '/joy': True}


# --- bag_stats ------------------------------------------------------------

def test_rates_match_nominal_within_what_jitter_allows(run):
    run_dir, truth = run
    stats = by_topic(bag_tools.stats(Bag(run_dir)))
    for topic in ['/drive', '/scan', '/sensors/core', '/joy']:
        s = stats[topic]
        jitter, lo, hi = spread(truth, topic)
        nominal = truth.rates[topic]
        # oracle: closed form -- (n-1) periods over a span that can be off by
        # at most 2 x stamp jitter + latency spread at its two ends
        slack = 2 * jitter + (hi - lo)
        span = (s['count'] - 1) / nominal
        periods = s['count'] - 1
        assert periods / (span + slack) <= s['rate_hz'] <= periods / (span - slack)
        assert s['period_median_s'] == pytest.approx(1 / nominal, abs=slack)   # s
        assert s['period_p99_s'] <= 1 / nominal + slack + US                   # jitter is bounded
        assert s['gap_count'] == 0


def test_jitter_is_actually_present_so_the_rate_test_above_is_not_trivial(run):
    stats = by_topic(bag_tools.stats(Bag(run[0])))
    # oracle: invariant -- with nonzero jitter, the p99 period exceeds the median
    # oracle: invariant
    assert stats['/drive']['period_p99_s'] > stats['/drive']['period_median_s'] + 0.001


def test_odom_dropout_is_found_with_its_length_and_start(run):
    run_dir, truth = run
    odom = by_topic(bag_tools.stats(Bag(run_dir)))['/odom']
    jitter, lo, hi = spread(truth, '/odom')
    period = 1 / truth.rates['/odom']
    start, end = truth.odom_dropout
    # oracle: closed form -- last message before is nominally start - period,
    # first after is nominally end, so the silence is (end - start) + period
    expected = (end - start) + period
    assert odom['worst_gap_s'] == pytest.approx(expected, abs=2 * jitter + (hi - lo) + US)
    gap_at_low = start - period - jitter + lo
    gap_at_high = start - period + jitter + hi
    assert gap_at_low - US <= odom['worst_gap_at_s'] <= gap_at_high + US


def test_lag_spike_is_found_and_also_shows_as_an_arrival_gap(run):
    run_dir, truth = run
    odom = by_topic(bag_tools.stats(Bag(run_dir)))['/odom']
    jitter, lo, hi = spread(truth, '/odom')
    spike_start = truth.lag_spike[0]
    # oracle: closed form -- the message stamped at the spike's start is
    # delayed by the full extra latency on top of its normal latency
    extra = truth.lag_spike_extra_s
    assert extra + lo - US <= odom['lag']['max_s'] <= extra + hi + US
    assert odom['lag']['max_at_s'] == pytest.approx(
        spike_start + truth.lag_spike_extra_s + (lo + hi) / 2, abs=jitter + (hi - lo) / 2 + US)
    # A stalled pipeline also goes quiet: nothing arrives for ~extra latency.
    period = 1 / truth.rates['/odom']
    assert odom['gap_count'] == 2
    stall = [g for g in odom['gaps'] if abs(g['at_s'] - spike_start) < 0.1]
    assert len(stall) == 1
    assert stall[0]['length_s'] == pytest.approx(truth.lag_spike_extra_s + period,
                                                 abs=2 * jitter + (hi - lo) + US)


def test_lag_of_clean_topics_stays_inside_the_generated_latency(run):
    run_dir, truth = run
    stats = by_topic(bag_tools.stats(Bag(run_dir)))
    for topic in ['/drive', '/scan', '/sensors/core', '/joy']:
        lo, hi = truth.latency_s[topic]
        lag = stats[topic]['lag']
        # oracle: invariant, latency is in [lo, hi]
        assert lo - US <= lag['median_s'] <= hi + US
        assert lag['max_s'] <= hi + US
        assert lag['negative_count'] == 0 and lag['stamp_backwards'] == 0


def test_worth_a_look_names_the_two_odom_problems_and_nothing_else(run):
    result = bag_tools.stats(Bag(run[0]))
    notes = bag_tools.worth_a_look(result)
    # oracle: invariant -- only /odom was given a dropout and a lag spike
    assert len(notes) == 2
    assert notes[0].startswith('/odom: 2 gap(s) over 3x')
    assert notes[1].startswith('/odom: data up to')


def test_a_single_message_topic_has_no_rate(run):
    stats = by_topic(bag_tools.stats(Bag(run[0])))
    s = stats['/robot_description']
    assert s['count'] == 1 and s['rate_hz'] is None and s['start_s'] == 0.0  # oracle: closed form


def test_stats_json_is_valid_json_with_the_notes(run, capsys):
    assert bag_tools.stats_main([str(run[0]), '--json', '--topics', '/odom']) == 0
    out = json.loads(capsys.readouterr().out)
    assert [t['topic'] for t in out['topics']] == ['/odom']
    assert len(out['worth_a_look']) == 2
    # oracle: spec, the documented default (docs/run-diagnostics.md)
    assert out['gap_factor'] == 3.0


# --- bag_find -------------------------------------------------------------

def test_battery_dip_without_a_filter_also_reports_every_spike(run):
    run_dir, truth = run
    result = bag_tools.find(Bag(run_dir), '/sensors/core', 'state.voltage_input < 10.5')
    # oracle: closed form -- one real dip plus one episode per injected spike
    assert result['total'] == 1 + len(truth.voltage_spike_stamps_s)
    spikes = [e for e in result['episodes'] if e['samples'] == 1]
    assert len(spikes) == len(truth.voltage_spike_stamps_s)
    for ep in spikes:
        assert ep['min'] == truth.voltage_spike_volts      # stored exactly


def test_battery_dip_with_min_duration_is_exactly_the_injected_event(run):
    run_dir, truth = run
    result = bag_tools.find(Bag(run_dir), '/sensors/core', 'state.voltage_input < 10.5',
                            min_duration_s=0.1)
    assert result['total'] == 1
    ep = result['episodes'][0]
    jitter, lo, hi = spread(truth, '/sensors/core')
    start, end = truth.battery_dip
    period = 1 / truth.rates['/sensors/core']
    # oracle: closed form -- nominal samples start .. end - period, each
    # arriving within [-jitter + lo, jitter + hi] of nominal
    assert start - jitter + lo - US <= ep['start_s'] <= start + jitter + hi + US
    assert end - period - jitter + lo - US <= ep['end_s'] <= end - period + jitter + hi + US
    assert ep['samples'] == round((end - start) * truth.rates['/sensors/core'])
    noise = 2 * truth.battery_noise_volts       # the generator's noise bound
    dip = truth.battery_dip_volts
    assert dip - noise <= ep['min'] <= ep['max'] <= dip + noise


def test_near_wall_is_found_through_nan_beams_and_glitches_are_filtered(run):
    run_dir, truth = run
    bag = Bag(run_dir)
    raw = bag_tools.find(bag, '/scan', 'min(ranges) < 0.4')
    # oracle: closed form -- the wall, plus one episode per single-beam glitch
    assert raw['total'] == 1 + len(truth.scan_glitch_stamps_s)
    filtered = bag_tools.find(bag, '/scan', 'min(ranges) < 0.4', min_duration_s=0.05)
    assert filtered['total'] == 1
    ep = filtered['episodes'][0]
    assert ep['samples'] == round((truth.wall[1] - truth.wall[0]) * truth.rates['/scan'])
    assert truth.wall_range_m - truth.scan_noise_m - US <= ep['min']   # float32 storage
    assert ep['max'] <= truth.wall_range_m + truth.scan_noise_m + US


def test_every_scan_has_bad_beams_so_nan_handling_is_exercised(run):
    rows = bag_tools.extract(Bag(run[0]), '/scan', ['nonfinite(ranges)'], hz=1.0, how='min')
    # oracle: invariant -- ~2% of 1081 beams are NaN/inf in every scan
    assert all(v[0] > 0 for _, v in rows)


def test_lb_release_is_found_on_the_joystick(run):
    run_dir, truth = run
    result = bag_tools.find(Bag(run_dir), '/joy', 'buttons[4] == 0')
    assert result['total'] == 1
    ep = result['episodes'][0]
    released_s = truth.lb_released[1] - truth.lb_released[0]
    assert ep['samples'] == round(released_s * truth.rates['/joy'])
    assert (ep['min'], ep['max']) == (0, 0)


def test_report_expression_is_summarised_per_episode(run):
    run_dir, truth = run
    result = bag_tools.find(Bag(run_dir), '/drive', 'drive.speed == 0',
                            report='drive.steering_angle')
    assert result['total'] == 1
    ep = result['episodes'][0]
    # oracle: closed form, wheelbase / circle radius in the generator
    steer = math.atan(0.36 / 3.0)
    assert ep['min'] == pytest.approx(steer, abs=1e-7)    # rad; float32 storage
    assert ep['max'] == pytest.approx(steer, abs=1e-7)


def test_find_respects_start_and_end(run):
    run_dir, truth = run
    result = bag_tools.find(Bag(run_dir), '/sensors/core', 'state.voltage_input < 10.5',
                            start_s=truth.battery_dip[1] + 0.1)
    # oracle: closed form -- only spikes after the dip remain
    later = [s for s in truth.voltage_spike_stamps_s if s > truth.battery_dip[1]]
    assert result['total'] == len(later)


# --- bag_extract ----------------------------------------------------------

def test_extract_buckets_show_the_dip_at_the_right_seconds(run):
    run_dir, truth = run
    rows = bag_tools.extract(Bag(run_dir), '/sensors/core', ['state.voltage_input'],
                             hz=1.0, start_s=18.0, end_s=24.0)
    assert [t for t, _ in rows] == [18.0, 19.0, 20.0, 21.0, 22.0, 23.0]  # oracle: closed form
    noise = 2 * truth.battery_noise_volts
    for t, (volts,) in rows:
        dipped = truth.battery_dip[0] <= t < truth.battery_dip[1]
        expected = truth.battery_dip_volts if dipped else truth.battery_nominal_volts
        # V; oracle: generator's levels + noise bound
        assert volts == pytest.approx(expected, abs=noise)


def test_extract_min_keeps_a_spike_that_mean_dilutes(run):
    run_dir, truth = run
    spike = truth.voltage_spike_stamps_s[0]
    bucket = math.floor(spike)
    bag = Bag(run_dir)
    lows = bag_tools.extract(bag, '/sensors/core', ['state.voltage_input'], hz=1.0,
                             how='min', start_s=bucket, end_s=bucket + 1)
    means = bag_tools.extract(bag, '/sensors/core', ['state.voltage_input'], hz=1.0,
                              start_s=bucket, end_s=bucket + 1)
    assert lows[0][1][0] == truth.voltage_spike_volts              # oracle: the injected value
    # oracle: invariant -- diluted above the threshold
    assert means[0][1][0] > 10.5


def test_extract_without_hz_returns_every_message_in_range(run):
    run_dir, truth = run
    rows = bag_tools.extract(Bag(run_dir), '/sensors/core', ['state.current_motor'],
                             start_s=18.0, end_s=24.0)
    # oracle: closed form -- 6 s at 50 Hz; jitter + latency (< 10 ms) never
    # carries a sample across a whole second boundary here
    assert len(rows) == 6 * truth.rates['/sensors/core']


def test_extract_cli_refuses_to_flood_the_screen_but_writes_a_file(run, tmp_path, capsys):
    run_dir, truth = run
    code = bag_tools.extract_main([str(run_dir), '/scan', 'min(ranges)'])
    captured = capsys.readouterr()
    assert code == 2 and captured.out == ''
    assert f'{truth.counts["/scan"]} rows is more than --max-rows 2000' in captured.err
    out = tmp_path / 'scan.csv'
    assert bag_tools.extract_main([str(run_dir), '/scan', 'min(ranges)', '-o', str(out)]) == 0
    rows = list(csv.reader(out.open()))
    assert rows[0] == ['t_s', 'min(ranges)']
    assert len(rows) == 1 + truth.counts['/scan']


def test_extract_cli_prints_csv(run, capsys):
    # --agg first: a message nominally at 50.00 s may arrive at 49.999 s and
    # land in the 49 s bucket, so a mean there is legitimately 0.95
    code = bag_tools.extract_main([str(run[0]), '/joy', 'buttons[4]', '--hz', '1',
                                   '--agg', 'first', '--start', '0:49', '--end', '0:53'])
    assert code == 0
    rows = list(csv.reader(io.StringIO(capsys.readouterr().out)))
    assert rows[0] == ['t_s', 'buttons[4]']
    # oracle: closed form -- LB held (1) at 49 and 52, released (0) through 50 and 51
    assert [(r[0], float(r[1])) for r in rows[1:]] == [
        ('49.000', 1.0), ('50.000', 0.0), ('51.000', 0.0), ('52.000', 1.0)]


def test_extract_a_whole_list_is_refused_with_a_hint(run, capsys):
    assert bag_tools.extract_main([str(run[0]), '/scan', 'ranges', '--hz', '1']) == 2
    assert 'is a list of 1081 values; reduce it' in capsys.readouterr().err


# --- bag_window -----------------------------------------------------------

def test_window_inside_the_dropout_has_no_odom_but_has_everything_else(run):
    run_dir, truth = run
    mid = sum(truth.odom_dropout) / 2
    result = bag_tools.window(Bag(run_dir), mid, 0.4)
    topics = {t['topic']: t for t in result['topics']}
    assert topics['/odom']['in_window'] == 0      # oracle: the injected dropout
    # oracle: closed form -- 0.4 s at 40 Hz is 16 messages, +-1 at the edges
    assert 15 <= topics['/drive']['in_window'] <= 17


def test_window_summarises_scans_and_caps_rows(run):
    run_dir, truth = run
    result = bag_tools.window(Bag(run_dir), sum(truth.wall) / 2, 0.5, topics=['/scan'], max_rows=3)
    scan = result['topics'][0]
    assert scan['shown'] == 3 and scan['in_window'] > 3
    for m in scan['messages']:
        ranges = m['msg']['ranges']
        assert ranges['n'] == truth.beams                       # summarised, not 1081 numbers
        assert ranges['min'] <= truth.wall_range_m + truth.scan_noise_m + US
        assert ranges.get('nan', 0) + ranges.get('inf', 0) > 0


def test_window_cli_prints_json_lines(run, capsys):
    # oracle: closed form
    assert bag_tools.window_main([str(run[0]), '--at', '0:30.5', '--span', '0.2',
                                  '--topics', '/drive', '--max-rows', '1']) == 0
    out = capsys.readouterr().out
    line = [ln for ln in out.splitlines() if ln.startswith('  30.')][0]
    msg = json.loads(line.split('s  ', 1)[1])
    assert msg['drive']['speed'] == 0.0           # oracle: /drive commands 0 at the wall


# --- errors: every one is a message and exit code 2, never a traceback -----

def test_missing_recording(tmp_path, capsys):
    assert bag_tools.stats_main([str(tmp_path / 'nope')]) == 2
    assert 'does not exist' in capsys.readouterr().err


def test_empty_directory(tmp_path, capsys):
    assert bag_tools.stats_main([str(tmp_path)]) == 2
    assert 'no recording (.mcap files) found' in capsys.readouterr().err


def test_sqlite_recording_is_named_as_such(tmp_path, capsys):
    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag' / 'bag_0.db3').write_bytes(b'SQLite format 3\x00')
    assert bag_tools.stats_main([str(tmp_path)]) == 2
    assert 'SQLite (.db3) format' in capsys.readouterr().err


def test_truncated_recording_is_reported_as_damaged(run, tmp_path, capsys):
    (tmp_path / 'bag').mkdir()
    data = (run[0] / 'bag' / 'bag_0.mcap').read_bytes()
    (tmp_path / 'bag' / 'bag_0.mcap').write_bytes(data[:len(data) // 2])
    assert bag_tools.stats_main([str(tmp_path)]) == 2
    assert 'is damaged or was cut off mid-write' in capsys.readouterr().err


def test_split_files_sort_numerically_not_alphabetically(run, tmp_path):
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    for i in [0, 1, 2, 10]:
        shutil.copy(run[0] / 'bag' / 'bag_0.mcap', bag_dir / f'bag_{i}.mcap')
    names = [f.name for f in Bag(tmp_path).files]
    assert names == ['bag_0.mcap', 'bag_1.mcap', 'bag_2.mcap', 'bag_10.mcap']


def test_unknown_topic_lists_the_real_ones(run, capsys):
    assert bag_tools.find_main([str(run[0]), '/battery', 'voltage < 10']) == 2
    err = capsys.readouterr().err
    assert 'not in this recording: /battery' in err and '/sensors/core' in err


def test_unknown_field_names_the_real_ones(run, capsys):
    assert bag_tools.find_main([str(run[0]), '/sensors/core', 'state.volts < 10']) == 2
    assert 'state has no field "volts"; available: temp_fet' in capsys.readouterr().err


def test_expression_that_tries_to_run_code_is_refused(run, capsys):
    assert bag_tools.find_main([str(run[0]), '/joy', "__import__('os').system('true')"]) == 2
    assert 'unknown function' in capsys.readouterr().err


@pytest.mark.parametrize('argv, message', [
    (['--min-duration', '-1'], 'cannot be negative'),
    (['--merge-gap', '-0.1'], 'cannot be negative'),
    (['--max-episodes', '0'], 'must be at least 1'),
])
def test_bad_find_options_are_refused(run, capsys, argv, message):
    assert bag_tools.find_main([str(run[0]), '/joy', 'buttons[4] == 0'] + argv) == 2
    assert message in capsys.readouterr().err


@pytest.mark.parametrize('argv, message', [
    (['--at', '-1'], 'is not a time'),
    (['--at', '1:75'], 'is not a time'),
])
def test_bad_times_are_refused_by_argparse(run, capsys, argv, message):
    with pytest.raises(SystemExit) as exc:
        bag_tools.window_main([str(run[0])] + argv)
    assert exc.value.code == 2
    assert message in capsys.readouterr().err


def test_bad_window_and_extract_options_are_refused(run, capsys):
    assert bag_tools.window_main([str(run[0]), '--at', '5', '--span', '0']) == 2
    assert '--span must be positive' in capsys.readouterr().err
    assert bag_tools.extract_main([str(run[0]), '/joy', 'buttons[4]', '--hz', '0']) == 2
    assert '--hz must be positive' in capsys.readouterr().err
    assert bag_tools.stats_main([str(run[0]), '--gap-factor', '1']) == 2
    assert '--gap-factor must be greater than 1' in capsys.readouterr().err


def test_bag_error_without_the_library_says_how_to_install(monkeypatch):
    # The one real process boundary here: whether a pip package is importable.
    import builtins
    real_import = builtins.__import__

    def no_mcap(name, *args, **kwargs):
        if name.startswith('mcap'):
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    from race_diagnostics import bag_io
    monkeypatch.setattr(builtins, '__import__', no_mcap)
    with pytest.raises(BagError, match='pip install --user --break-system-packages mcap==1.6.0'):
        bag_io._require_mcap()
