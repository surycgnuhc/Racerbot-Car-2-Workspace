"""
session.py

Where a logging session goes on disk and what its files look like.

    <workspace>/data_logs/<category>/<YYYYmmdd-HHMMSS>/
        metadata.json   category, notes, source (car/sim), topics, units, counts
        imu.csv         one row per /sensors/imu/raw message
        scan.csv        one row per /scan message, one column per beam
        odom.csv        one row per /odom message (what the labels come from)

Every file is opened line-buffered, so a hard kill -- or a car switched
off in a hurry -- still leaves every completed row on disk.

Pure Python, no rclpy: the node hands plain numbers in, and the tests
exercise the real file format without ROS.
"""

import csv
import json
import math
import os
from pathlib import Path

from .maneuvers import validate_category

DATA_LOG_DIRNAME = 'data_logs'

IMU_COLUMNS = ('stamp', 'recv_time', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
               'qw', 'qx', 'qy', 'qz')
ODOM_COLUMNS = ('stamp', 'recv_time', 'x', 'y', 'yaw', 'speed', 'yaw_rate')
SCAN_FIXED_COLUMNS = ('stamp', 'recv_time', 'angle_min', 'angle_increment',
                      'range_min', 'range_max')

# What /sensors/imu/raw actually carries. vesc_driver copies the VESC's
# own units straight into sensor_msgs/Imu without converting
# (vesc_driver.cpp, publish of std_imu_msg), and the VESC reports
# acceleration in g and angular rate in deg/s (vesc_packet.cpp,
# VescPacketImu::acc_x / gyr_x) -- NOT the m/s^2 and rad/s that
# sensor_msgs/Imu specifies.
GRAVITY = {'g': 1.0, 'm/s^2': 9.80665}

# Was this the real car or racerbot_sim? Decided by WHO publishes the
# topics being recorded, not by hostname or a flag someone can forget to
# set. Node names as the launch files set them (test_node_static.py checks
# them against those files):
#   car: bringup_launch.py / sick_bringup_launch.py / no_lidar_bringup_launch.py
#   sim: racerbot_sim/launch/sim_bringup_launch.py
CAR_SENSOR_NODES = frozenset({'urg_node', 'sick_node', 'vesc_to_odom_node',
                              'vesc_driver_node'})
SIM_SENSOR_NODES = frozenset({'gym_bridge_node'})

SOURCE_CAR = 'car'
SOURCE_SIM = 'sim'
SOURCE_MIXED = 'mixed'        # more than one kind: do not trust this data
SOURCE_UNKNOWN = 'unknown'    # no publisher seen at all
SOURCES = (SOURCE_CAR, SOURCE_SIM, SOURCE_MIXED, SOURCE_UNKNOWN)


def classify_source(publisher_nodes) -> str:
    """'car', 'sim', 'mixed' or 'unknown' from the names of the nodes seen
    publishing the recorded topics.

    Anything that is neither a known car driver nor the simulator (a
    `ros2 bag play`, a `ros2 topic pub`, a node someone wrote) counts as a
    third kind. On its own it is 'unknown'; alongside the car or the sim it
    makes the session 'mixed', because two sources were then writing into
    the same topic and the recording cannot be attributed to either.
    """
    kinds = set()
    for name in publisher_nodes:
        if name in CAR_SENSOR_NODES:
            kinds.add(SOURCE_CAR)
        elif name in SIM_SENSOR_NODES:
            kinds.add(SOURCE_SIM)
        else:
            kinds.add('other')
    if len(kinds) > 1:
        return SOURCE_MIXED
    if kinds == {SOURCE_CAR}:
        return SOURCE_CAR
    if kinds == {SOURCE_SIM}:
        return SOURCE_SIM
    return SOURCE_UNKNOWN


def find_workspace_root(start):
    """The nearest directory at or above `start` holding both src/ and
    install/ -- a built colcon workspace. None if there is none.

    The node passes its own installed share directory, which always sits
    at <workspace>/install/sensor_logger/share/sensor_logger.
    """
    path = Path(start).absolute()
    for candidate in (path, *path.parents):
        if (candidate / 'src').is_dir() and (candidate / 'install').is_dir():
            return candidate
    return None


def make_session_dir(log_root, category: str, stamp: str) -> Path:
    """Create and return log_root/<category>/<stamp>, never reusing one.

    Two sessions started in the same second get <stamp>-2, <stamp>-3...
    `category` must be in the catalogue: a typo would otherwise scatter
    recordings across folders nobody looks in.
    """
    validate_category(category)
    parent = Path(log_root) / category
    parent.mkdir(parents=True, exist_ok=True)
    name, n = stamp, 1
    while True:
        try:
            path = parent / name
            path.mkdir()
            return path
        except FileExistsError:
            n += 1
            name = f'{stamp}-{n}'


def yaw_from_quaternion(w, x, y, z) -> float:
    """Heading (rotation about z) in radians, in (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def write_json_atomic(path, data):
    """Write JSON so a reader never sees a half-written file."""
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'w') as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
        handle.write('\n')
    os.replace(tmp, path)


class CsvLog:
    """A CSV file with a fixed header, flushed line by line."""

    def __init__(self, path, columns):
        self.columns = tuple(columns)
        self.rows = 0
        self._file = open(path, 'w', buffering=1, newline='')
        self._writer = csv.writer(self._file, lineterminator='\n')
        self._writer.writerow(self.columns)

    def write(self, row):
        if len(row) != len(self.columns):
            raise ValueError(
                f'row has {len(row)} values, header has {len(self.columns)}')
        self._writer.writerow(row)
        self.rows += 1

    def close(self):
        self._file.close()


class ScanLog:
    """scan.csv: the fixed columns, then r0..r<N-1>, one per beam.

    The beam count is fixed by the first non-empty scan. A later scan with
    a different count cannot fit the header, so it is rejected and counted
    rather than written misaligned -- the UST-10LX always sends the same
    count, so a non-zero `rejected` means something upstream changed.

    Ranges are written to 1 mm (3 decimals). urg_node reports the Hokuyo's
    integer millimetres divided by 1000, so this loses nothing the sensor
    measured and keeps a 1081-beam row near 7 kB instead of 20 kB.
    """

    def __init__(self, path):
        self.beams = None
        self.rows = 0
        self.rejected = 0
        self._file = open(path, 'w', buffering=1, newline='')
        self._writer = csv.writer(self._file, lineterminator='\n')

    def write(self, stamp, recv_time, angle_min, angle_increment,
              range_min, range_max, ranges) -> bool:
        if self.beams is None:
            if len(ranges) == 0:
                self.rejected += 1
                return False
            self.beams = len(ranges)
            self._writer.writerow(
                SCAN_FIXED_COLUMNS + tuple(f'r{i}' for i in range(self.beams)))
        elif len(ranges) != self.beams:
            self.rejected += 1
            return False
        self._writer.writerow(
            [stamp, recv_time, angle_min, angle_increment, range_min, range_max]
            + [f'{r:.3f}' for r in ranges])
        self.rows += 1
        return True

    def close(self):
        self._file.close()


class ImuStats:
    """Running sanity check on IMU samples, so a dead IMU is reported
    while recording instead of discovered later in a folder of zeros.

    verdict():
      no_data        nothing received
      all_zero       every accel and gyro value exactly 0.0 -- the
                     documented VESC fault (docs/hardware-reference.md)
      unit_mismatch  mean |accel| is gravity in the OTHER unit
      no_gravity     mean |accel| is not gravity in either unit
      ok
    Gravity dominates |accel| even while driving (a full-lock corner at
    1 m/s adds 0.74 m/s^2 sideways, under 0.3% of the norm), so the mean
    over a whole session is a fair test.
    """

    def __init__(self, accel_unit: str = 'g', tolerance_fraction: float = 0.15):
        if accel_unit not in GRAVITY:
            raise ValueError(
                f'unknown accel unit {accel_unit!r}; expected one of: {", ".join(GRAVITY)}')
        self.accel_unit = accel_unit
        self.tolerance_fraction = tolerance_fraction
        self.count = 0
        self.invalid = 0
        self.nonzero = 0
        self._norm_sum = 0.0

    def add(self, ax, ay, az, gx, gy, gz):
        values = (ax, ay, az, gx, gy, gz)
        if not all(math.isfinite(v) for v in values):
            self.invalid += 1
            return
        self.count += 1
        if any(v != 0.0 for v in values):
            self.nonzero += 1
        self._norm_sum += math.sqrt(ax * ax + ay * ay + az * az)

    def mean_accel_norm(self):
        return self._norm_sum / self.count if self.count else None

    def _is_gravity(self, norm, unit):
        return abs(norm - GRAVITY[unit]) <= self.tolerance_fraction * GRAVITY[unit]

    def verdict(self) -> str:
        if self.count == 0:
            return 'no_data'
        if self.nonzero == 0:
            return 'all_zero'
        norm = self.mean_accel_norm()
        if self._is_gravity(norm, self.accel_unit):
            return 'ok'
        if any(self._is_gravity(norm, u) for u in GRAVITY if u != self.accel_unit):
            return 'unit_mismatch'
        return 'no_gravity'

    def as_dict(self):
        norm = self.mean_accel_norm()
        return {'samples': self.count, 'invalid': self.invalid,
                'nonzero': self.nonzero, 'accel_unit': self.accel_unit,
                'mean_accel_norm': None if norm is None else round(norm, 4),
                'verdict': self.verdict()}
