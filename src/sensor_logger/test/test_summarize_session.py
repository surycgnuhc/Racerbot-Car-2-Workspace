"""
End-to-end tests for summarize_session: a session folder written with the
same writers the node uses, then summarized from disk.

    python3 -m pytest src/sensor_logger/test/ -v
"""
import csv
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sensor_logger.session import (  # noqa: E402
    IMU_COLUMNS, ODOM_COLUMNS, CsvLog, ScanLog, write_json_atomic)
from sensor_logger.summarize_session import main  # noqa: E402

FULL_LOCK = 0.74   # 1/m, docs/hardware-reference.md


def write_session(root, category, odom, imu=None, scans=3, unit='g', meta=None):
    """odom: list of (stamp, speed, yaw_rate). imu: list of 6-tuples."""
    # Built by hand, not make_session_dir: that refuses unknown categories,
    # and one test needs metadata naming one.
    path = root / category / '20261016-141500'
    path.mkdir(parents=True)
    write_json_atomic(path / 'metadata.json', {
        'category': category, 'notes': 'test', 'units': {'imu_accel': unit},
        **(meta or {})})
    log = CsvLog(path / 'odom.csv', ODOM_COLUMNS)
    for stamp, speed, yaw_rate in odom:
        log.write((stamp, stamp, 0.0, 0.0, 0.0, speed, yaw_rate))
    log.close()
    log = CsvLog(path / 'imu.csv', IMU_COLUMNS)
    for sample in imu or []:
        log.write((0.0, 0.0) + tuple(sample) + (1.0, 0.0, 0.0, 0.0))
    log.close()
    scan = ScanLog(path / 'scan.csv')
    for i in range(scans):           # 40 Hz, 4 beams, one of them inf
        scan.write(0.025 * i, 0.0, -1.0, 0.5, 0.02, 30.0, [1.0, 2.0, float('inf'), 3.0])
    scan.close()
    return path


def left_turn(seconds=3.0, rate=50):
    return [(i / rate, 1.0, FULL_LOCK) for i in range(int(seconds * rate))]


def _summary(path):
    return json.loads((path / 'summary.json').read_text())


def test_a_left_turn_session_passes_as_turn_left(tmp_path, capsys):
    path = write_session(tmp_path, 'turn_left', left_turn(),
                         imu=[(0.0,) * 6] * 50)
    assert main([str(path)]) == 0
    summary = _summary(path)
    assert summary['check']['passed'] is True
    assert summary['check']['label_seconds'] == {
        'turn_left': pytest.approx(149 / 50, abs=1e-9)}  # s, 150 samples
    assert 'turn_left: PASS' in capsys.readouterr().out


def test_the_same_data_filed_as_turn_right_fails(tmp_path, capsys):
    path = write_session(tmp_path, 'turn_right', left_turn())
    assert main([str(path)]) == 1
    failures = _summary(path)['check']['failures']
    assert failures == ['turn_right: 0.00s, needs at least 2.00s',
                        'turn_left: 2.98s, allowed at most 0.50s']
    assert 'turn_right: FAIL' in capsys.readouterr().out


def test_labels_csv_has_one_row_per_odom_sample(tmp_path):
    odom = [(0.0, 0.0, 0.0), (0.25, 0.0, 0.0), (0.5, 1.0, 0.0),
            (0.75, 1.0, 1.0), (1.0, -1.0, 0.0)]
    path = write_session(tmp_path, 'free_drive', odom)
    assert main([str(path)]) == 0
    with open(path / 'labels.csv', newline='') as handle:
        rows = list(csv.reader(handle))
    # t=0.5: 1 m/s gained over 0.25 s = 4 m/s^2 -> accelerating.
    # t=0.75: flat over the window, curvature 1.0 -> left.
    # t=1.0: backing up.
    assert rows == [['stamp', 'label'], ['0.0', 'idle'], ['0.25', 'idle'],
                    ['0.5', 'accelerating'], ['0.75', 'turn_left'],
                    ['1.0', 'reverse']]


def test_a_dead_imu_is_reported_as_all_zero(tmp_path):
    path = write_session(tmp_path, 'idle', [(i * 0.25, 0.0, 0.0) for i in range(41)],
                         imu=[(0.0,) * 6] * 500)
    assert main([str(path)]) == 0
    imu = _summary(path)['imu']
    assert (imu['verdict'], imu['samples'], imu['nonzero']) == ('all_zero', 500, 0)


def test_a_working_imu_in_g_is_ok(tmp_path):
    path = write_session(tmp_path, 'free_drive', [(0.0, 0.0, 0.0)],
                         imu=[(0.0, 0.0, 1.0, 0.0, 0.0, 0.5)] * 10, unit='g')
    main([str(path)])
    assert _summary(path)['imu']['verdict'] == 'ok'


def test_scan_health_rate_and_finite_fraction(tmp_path):
    path = write_session(tmp_path, 'free_drive', [(0.0, 0.0, 0.0)], scans=5)
    main([str(path)])
    scan = _summary(path)['scan']
    # 5 scans 0.025 s apart: 4 gaps over 0.1 s = 40 Hz; 1 of 4 beams is inf.
    assert scan == {'scans': 5, 'beams': 4, 'malformed': 0,
                    'rate_hz': 40.0, 'finite_fraction': 0.75}


def test_a_line_cut_by_a_hard_kill_is_skipped_not_misread(tmp_path):
    """A kill mid-write leaves '...,0.5' where '0.512' was meant: the right
    column count, so only the missing newline gives it away."""
    path = write_session(tmp_path, 'turn_left', left_turn())
    with open(path / 'odom.csv', 'a') as handle:
        handle.write('3.0,3.0,0.0,0.0,0.0,9.0,0.5')       # no newline
    assert main([str(path)]) == 0
    summary = _summary(path)
    assert summary['odom'] == {'samples': 150, 'malformed': 1}
    assert summary['check']['peak_speed_mps'] == 1.0     # 9.0 m/s never read


@pytest.mark.parametrize('line', ['1.0,1.0,0.0\n', '1.0,1.0,0,0,0,fast,0\n',
                                  '1.0,1.0,0,0,0,1,0,extra\n'])
def test_malformed_rows_are_counted_and_skipped(tmp_path, line):
    path = write_session(tmp_path, 'free_drive', [(0.0, 0.0, 0.0)])
    with open(path / 'odom.csv', 'a') as handle:
        handle.write(line)
    main([str(path)])
    assert _summary(path)['odom'] == {'samples': 1, 'malformed': 1}


def test_free_drive_is_unchecked(tmp_path, capsys):
    path = write_session(tmp_path, 'free_drive', left_turn())
    assert main([str(path)]) == 0
    assert 'UNCHECKED' in capsys.readouterr().out


def test_json_output_is_the_summary_file(tmp_path, capsys):
    path = write_session(tmp_path, 'turn_left', left_turn())
    main([str(path), '--json'])
    assert json.loads(capsys.readouterr().out) == _summary(path)


def test_an_empty_session_fails_its_check_but_still_summarizes(tmp_path):
    path = write_session(tmp_path, 'stop', [], scans=0)
    assert main([str(path)]) == 1
    summary = _summary(path)
    assert summary['check']['failures'] == ['only 0 odometry sample(s); nothing to check']
    assert summary['imu']['verdict'] == 'no_data'
    assert summary['scan']['scans'] == 0


def test_not_a_session_folder_exits_2(tmp_path, capsys):
    assert main([str(tmp_path)]) == 2
    assert 'no metadata.json' in capsys.readouterr().err


def test_unknown_category_in_metadata_exits_2(tmp_path, capsys):
    path = write_session(tmp_path, 'turnleft', left_turn())
    assert main([str(path)]) == 2
    assert "unknown category 'turnleft'" in capsys.readouterr().err


def test_time_running_backwards_exits_2(tmp_path, capsys):
    path = write_session(tmp_path, 'free_drive', [(1.0, 0.0, 0.0), (0.5, 0.0, 0.0)])
    assert main([str(path)]) == 2
    assert 'non-decreasing' in capsys.readouterr().err


def test_missing_column_exits_2(tmp_path, capsys):
    path = write_session(tmp_path, 'free_drive', [(0.0, 0.0, 0.0)])
    (path / 'odom.csv').write_text('stamp,speed\n0.0,0.0\n')
    assert main([str(path)]) == 2
    assert 'missing column' in capsys.readouterr().err


# --- source: car or sim ------------------------------------------------------

def test_source_and_its_publishers_are_carried_into_the_summary(tmp_path, capsys):
    publishers = {'imu': [], 'scan': ['gym_bridge_node'], 'odom': ['gym_bridge_node']}
    path = write_session(tmp_path, 'turn_left', left_turn(),
                         meta={'source': 'sim', 'source_publishers': publishers})
    assert main([str(path)]) == 0
    summary = _summary(path)
    assert (summary['source'], summary['source_publishers']) == ('sim', publishers)
    assert '  source    sim (gym_bridge_node)\n' in capsys.readouterr().out


def test_a_car_session_lists_every_publisher_once(tmp_path, capsys):
    publishers = {'imu': ['vesc_driver_node'], 'scan': ['urg_node'],
                  'odom': ['vesc_to_odom_node']}
    path = write_session(tmp_path, 'turn_left', left_turn(),
                         meta={'source': 'car', 'source_publishers': publishers})
    main([str(path)])
    assert ('  source    car (urg_node, vesc_driver_node, vesc_to_odom_node)\n'
            in capsys.readouterr().out)


def test_a_recording_from_before_the_field_is_unknown(tmp_path, capsys):
    path = write_session(tmp_path, 'turn_left', left_turn())
    assert main([str(path)]) == 0
    summary = _summary(path)
    assert (summary['source'], summary['source_publishers']) == ('unknown', {})
    assert '  source    unknown\n' in capsys.readouterr().out


@pytest.mark.parametrize('meta,error', [
    ({'source': 'real'}, "source 'real' is not one of car, sim, mixed, unknown"),
    ({'source': 'car', 'source_publishers': {'scan': 'urg_node'}},
     'source_publishers must map each topic to a list of node names'),
    ({'source': 'car', 'source_publishers': ['urg_node']},
     'source_publishers must map each topic to a list of node names'),
    ({'source': 'car', 'source_publishers': {'scan': [7]}},
     'source_publishers must map each topic to a list of node names'),
])
def test_a_bad_source_in_metadata_exits_2(tmp_path, capsys, meta, error):
    path = write_session(tmp_path, 'turn_left', left_turn(), meta=meta)
    assert main([str(path)]) == 2
    assert error in capsys.readouterr().err
    assert not (path / 'summary.json').exists()
