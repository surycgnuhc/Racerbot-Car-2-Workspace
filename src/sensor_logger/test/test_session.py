"""
Tests for session.py: where sessions go, the on-disk formats, and the
IMU sanity check. Real files in pytest's tmp_path -- nothing mocked.

    python3 -m pytest src/sensor_logger/test/ -v
"""
import csv
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sensor_logger.session import (  # noqa: E402
    CAR_SENSOR_NODES, IMU_COLUMNS, SCAN_FIXED_COLUMNS, SIM_SENSOR_NODES,
    CsvLog, ImuStats, ScanLog, classify_source, find_workspace_root,
    make_session_dir, write_json_atomic, yaw_from_quaternion)


def _read(path):
    with open(path, newline='') as handle:
        return list(csv.reader(handle))


# --- find_workspace_root -------------------------------------------------

def test_finds_the_workspace_from_the_installed_share_dir(tmp_path):
    ws = tmp_path / 'racerbot-ws'
    share = ws / 'install' / 'sensor_logger' / 'share' / 'sensor_logger'
    share.mkdir(parents=True)
    (ws / 'src').mkdir()
    assert find_workspace_root(share) == ws


def test_nearest_workspace_wins(tmp_path):
    outer = tmp_path
    (outer / 'src').mkdir()
    (outer / 'install').mkdir()
    inner = outer / 'nested'
    (inner / 'src').mkdir(parents=True)
    (inner / 'install' / 'x').mkdir(parents=True)
    assert find_workspace_root(inner / 'install' / 'x') == inner


@pytest.mark.parametrize('present', [(), ('src',), ('install',)])
def test_no_workspace_is_none(tmp_path, present):
    for name in present:
        (tmp_path / name).mkdir()
    start = tmp_path / 'deep' / 'er'
    start.mkdir(parents=True)
    # tmp_path's own parents (/tmp...) are not a workspace either.
    assert find_workspace_root(start) is None


# --- make_session_dir ----------------------------------------------------

def test_session_dir_is_category_then_stamp(tmp_path):
    path = make_session_dir(tmp_path, 'turn_left', '20261016-141500')
    assert path == tmp_path / 'turn_left' / '20261016-141500'
    assert path.is_dir()


def test_same_second_never_reuses_a_folder(tmp_path):
    paths = [make_session_dir(tmp_path, 'stop', '20261016-141500') for _ in range(3)]
    assert [p.name for p in paths] == [
        '20261016-141500', '20261016-141500-2', '20261016-141500-3']


@pytest.mark.parametrize('bad', ['turnleft', 'Turn_Left', '', '../etc', 'idle/x'])
def test_unknown_category_creates_nothing(tmp_path, bad):
    with pytest.raises(ValueError, match='unknown category'):
        make_session_dir(tmp_path, bad, '20261016-141500')
    assert list(tmp_path.iterdir()) == []


# --- yaw_from_quaternion -------------------------------------------------

@pytest.mark.parametrize('yaw', [-3.0, -1.0, 0.0, 0.5, 1.5, 3.0])
def test_yaw_of_a_pure_z_rotation(yaw):
    # Closed form: rotation by yaw about z is (cos(yaw/2), 0, 0, sin(yaw/2)).
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    assert yaw_from_quaternion(*q) == pytest.approx(yaw, abs=1e-12)  # rad


def test_yaw_survives_roll():
    """Yaw 1.0 then roll 0.8 (ZYX, zero pitch): yaw must read back as 1.0.
    q = qz(1.0) * qx(0.8), multiplied out by hand:
    w = cz cx, x = cz sx, y = sz sx, z = sz cx (c, s of the half angles)."""
    cz, sz = math.cos(0.5), math.sin(0.5)
    cx, sx = math.cos(0.4), math.sin(0.4)
    q = (cz * cx, cz * sx, sz * sx, sz * cx)
    assert yaw_from_quaternion(*q) == pytest.approx(1.0, abs=1e-12)  # rad


def test_roll_alone_is_no_yaw():
    q = (math.cos(0.4), math.sin(0.4), 0.0, 0.0)   # 0.8 rad about x
    assert yaw_from_quaternion(*q) == pytest.approx(0.0, abs=1e-12)  # rad


# --- write_json_atomic ---------------------------------------------------

def test_json_is_replaced_whole_and_no_temp_is_left(tmp_path):
    path = tmp_path / 'metadata.json'
    write_json_atomic(path, {'a': 1})
    write_json_atomic(path, {'b': [1.5, None]})
    assert json.loads(path.read_text()) == {'b': [1.5, None]}
    assert [p.name for p in tmp_path.iterdir()] == ['metadata.json']


# --- CsvLog ---------------------------------------------------------------

def test_csv_rows_round_trip_exactly_including_non_finite(tmp_path):
    path = tmp_path / 'imu.csv'
    log = CsvLog(path, IMU_COLUMNS)
    row = (1787452767.875881949, 0.1 + 0.2, -0.0, math.inf, -math.inf,
           math.nan, 1e-300, 2.5, 1.0, 0.0, 0.0, 0.0)
    log.write(row)
    log.close()
    header, written = _read(path)
    assert tuple(header) == IMU_COLUMNS
    back = [float(v) for v in written]
    for got, want in zip(back, row):
        assert got == want or (math.isnan(got) and math.isnan(want))
    assert log.rows == 1


def test_csv_lines_end_in_newline_so_a_cut_line_is_detectable(tmp_path):
    path = tmp_path / 'o.csv'
    log = CsvLog(path, ('a', 'b'))
    log.write((1, 2))
    log.close()
    assert path.read_bytes() == b'a,b\n1,2\n'


@pytest.mark.parametrize('n', [0, 11, 13])
def test_csv_refuses_a_row_that_does_not_fit_the_header(tmp_path, n):
    log = CsvLog(tmp_path / 'imu.csv', IMU_COLUMNS)
    with pytest.raises(ValueError, match=f'row has {n} values, header has 12'):
        log.write((0.0,) * n)
    log.close()
    assert len(_read(tmp_path / 'imu.csv')) == 1     # header only


# --- ScanLog ----------------------------------------------------------------

def test_scan_header_has_one_column_per_beam(tmp_path):
    log = ScanLog(tmp_path / 'scan.csv')
    assert log.write(1.0, 2.0, -2.35, 0.0044, 0.02, 30.0, [1.234, math.inf, math.nan, 0.0])
    log.close()
    header, row = _read(tmp_path / 'scan.csv')
    assert tuple(header) == SCAN_FIXED_COLUMNS + ('r0', 'r1', 'r2', 'r3')
    assert row[6:] == ['1.234', 'inf', 'nan', '0.000']
    assert [float(v) for v in row[:6]] == [1.0, 2.0, -2.35, 0.0044, 0.02, 30.0]


@pytest.mark.parametrize('r', [0.0004, 1.2344, 1.2346, 29.9995, 7.0])
def test_scan_ranges_are_kept_to_the_millimetre(tmp_path, r):
    log = ScanLog(tmp_path / 'scan.csv')
    log.write(0.0, 0.0, 0.0, 0.1, 0.0, 30.0, [r])
    log.close()
    written = float(_read(tmp_path / 'scan.csv')[1][6])
    assert abs(written - r) <= 0.0005 + 1e-12      # half a millimetre, in m


def test_scan_with_a_different_beam_count_is_rejected_not_misaligned(tmp_path):
    log = ScanLog(tmp_path / 'scan.csv')
    assert log.write(0.0, 0.0, 0.0, 0.1, 0.0, 30.0, [1.0] * 4)
    assert not log.write(0.1, 0.0, 0.0, 0.1, 0.0, 30.0, [1.0] * 3)
    assert not log.write(0.2, 0.0, 0.0, 0.1, 0.0, 30.0, [1.0] * 5)
    assert log.write(0.3, 0.0, 0.0, 0.1, 0.0, 30.0, [2.0] * 4)
    log.close()
    rows = _read(tmp_path / 'scan.csv')
    assert (log.rows, log.rejected, log.beams) == (2, 2, 4)
    assert [r[0] for r in rows[1:]] == ['0.0', '0.3']
    assert all(len(r) == 10 for r in rows)


def test_an_empty_first_scan_does_not_fix_the_header(tmp_path):
    log = ScanLog(tmp_path / 'scan.csv')
    assert not log.write(0.0, 0.0, 0.0, 0.1, 0.0, 30.0, [])
    assert log.write(0.1, 0.0, 0.0, 0.1, 0.0, 30.0, [1.0, 2.0])
    log.close()
    header = _read(tmp_path / 'scan.csv')[0]
    assert header[-2:] == ['r0', 'r1'] and log.beams == 2 and log.rejected == 1


def test_no_scans_leaves_an_empty_file(tmp_path):
    ScanLog(tmp_path / 'scan.csv').close()
    assert (tmp_path / 'scan.csv').read_bytes() == b''


# --- ImuStats ----------------------------------------------------------------

def _stats(samples, unit='g'):
    stats = ImuStats(unit)
    for s in samples:
        stats.add(*s)
    return stats


def test_all_zeros_is_the_documented_vesc_fault():
    """docs/hardware-reference.md, measured 2026-08-19: every field 0.0."""
    assert _stats([(0.0,) * 6] * 100).verdict() == 'all_zero'


def test_no_samples_is_no_data():
    assert _stats([]).verdict() == 'no_data'
    assert _stats([(math.nan,) * 6]).verdict() == 'no_data'


@pytest.mark.parametrize('sample,unit,expected', [
    ((0.0, 0.0, 1.0, 0.0, 0.0, 0.0), 'g', 'ok'),               # at rest, in g
    ((0.0, 0.0, 9.80665, 0.0, 0.0, 0.0), 'm/s^2', 'ok'),
    ((0.0, 0.0, 9.80665, 0.0, 0.0, 0.0), 'g', 'unit_mismatch'),
    ((0.0, 0.0, 1.0, 0.0, 0.0, 0.0), 'm/s^2', 'unit_mismatch'),
    ((0.6, 0.0, 0.8, 0.0, 0.0, 0.0), 'g', 'ok'),               # |a| = 1, tilted
    ((0.0, 0.0, 0.5, 0.0, 0.0, 0.0), 'g', 'no_gravity'),
    ((0.0, 0.0, 0.0, 0.0, 0.0, 3.0), 'g', 'no_gravity'),        # gyro alive, accel not
])
def test_gravity_check(sample, unit, expected):
    assert _stats([sample] * 10, unit).verdict() == expected


# Just inside / just outside the 15% band, in both units. Not "exactly
# 0.85": in binary 1.0 - 0.85 is 0.15000000000000002, so that literal is
# not on the bound. 9.80665 m/s^2 is standard gravity (CGPM 1901).
@pytest.mark.parametrize('unit,g', [('g', 1.0), ('m/s^2', 9.80665)])
@pytest.mark.parametrize('factor,expected', [
    (1.1499, 'ok'),
    (1.1501, 'no_gravity'),
    (0.8501, 'ok'),
    (0.8499, 'no_gravity'),
])
def test_gravity_tolerance_boundary(unit, g, factor, expected):
    sample = (0.0, 0.0, factor * g, 0.0, 0.0, 0.0)
    assert _stats([sample], unit).verdict() == expected


def test_non_finite_samples_are_counted_and_kept_out_of_the_mean():
    stats = _stats([(0.0, 0.0, 1.0, 0.0, 0.0, 0.0), (math.inf, 0.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 1.0, math.nan, 0.0, 0.0)])
    assert (stats.count, stats.invalid) == (1, 2)
    assert stats.mean_accel_norm() == 1.0
    assert stats.verdict() == 'ok'


def test_mean_is_over_all_samples():
    stats = _stats([(0.0, 0.0, 0.9, 0.0, 0.0, 0.0), (0.0, 0.0, 1.1, 0.0, 0.0, 0.0)])
    assert stats.mean_accel_norm() == pytest.approx(1.0, abs=1e-12)  # g


def test_unknown_unit_is_refused():
    with pytest.raises(ValueError, match="unknown accel unit 'G'"):
        ImuStats('G')


# --- classify_source ---------------------------------------------------------
# Oracle: the rule in classify_source's docstring, applied by hand. Node
# names are the ones the launch files set (test_node_static.py checks them).

@pytest.mark.parametrize('publishers,expected', [
    ((), 'unknown'),
    (('urg_node', 'vesc_to_odom_node', 'vesc_driver_node'), 'car'),   # bringup
    (('sick_node', 'vesc_to_odom_node', 'vesc_driver_node'), 'car'),  # sick_bringup
    (('vesc_to_odom_node',), 'car'),                                   # no_lidar_bringup
    (('gym_bridge_node',), 'sim'),
    (('gym_bridge_node', 'urg_node'), 'mixed'),          # sim beside the real LiDAR
    (('gym_bridge_node', 'vesc_to_odom_node'), 'mixed'),
    (('rosbag2_player',), 'unknown'),                    # a replay is not the car
    (('urg_node', 'rosbag2_player'), 'mixed'),           # replay injected into live data
    (('gym_bridge_node', '_ros2cli_1234'), 'mixed'),     # `ros2 topic pub` into the sim
    (('urg_node_driver',), 'unknown'),                   # the executable, not the node name
    (('gym_bridge',), 'unknown'),
])
def test_source_is_decided_by_who_publishes(publishers, expected):
    assert classify_source(publishers) == expected


def test_source_ignores_order_and_duplicates():
    names = ['vesc_driver_node', 'urg_node', 'urg_node', 'vesc_to_odom_node']
    assert classify_source(names) == classify_source(reversed(names)) == 'car'


def test_no_node_is_both_car_and_sim():
    assert CAR_SENSOR_NODES & SIM_SENSOR_NODES == set()
