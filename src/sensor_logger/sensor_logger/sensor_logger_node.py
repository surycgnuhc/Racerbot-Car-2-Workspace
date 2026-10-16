"""
sensor_logger_node.py

Records IMU, LiDAR and odometry into one session folder per run, filed
under the test-vector category the driver chose:

    ros2 launch sensor_logger sensor_logger_launch.py category:=turn_left

This node ONLY subscribes. It never publishes to any topic, so it cannot
move the car and is support tooling, not driving code (see
docs/adding-your-own-code.md) -- test_node_static.py fails the build if a
publisher ever appears here. Driving the maneuver is the control layer's
job (teleop_launch.py, LB held); this just writes down what happened.

All file formats and checks live in session.py and maneuvers.py, which do
not import rclpy. This file only converts messages to numbers.
"""

import socket
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

from .maneuvers import CATEGORIES, validate_category
from .session import (DATA_LOG_DIRNAME, IMU_COLUMNS, ODOM_COLUMNS,
                      SOURCE_MIXED, CsvLog, ImuStats, ScanLog, classify_source,
                      find_workspace_root, make_session_dir, write_json_atomic,
                      yaw_from_quaternion)


def _stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class SensorLoggerNode(Node):

    def __init__(self):
        super().__init__('sensor_logger_node')

        self.declare_parameter('category', 'free_drive')
        self.declare_parameter('notes', '')
        self.declare_parameter('log_root', '')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_accel_unit', 'g')
        self.declare_parameter('imu_gyro_unit', 'deg/s')
        self.declare_parameter('status_period_sec', 5.0)

        def value(name):
            return self.get_parameter(name).value

        self.category = str(value('category'))
        spec = validate_category(self.category)   # unknown -> refuse to start

        log_root = str(value('log_root'))
        if not log_root:
            workspace = find_workspace_root(get_package_share_directory('sensor_logger'))
            if workspace is None:
                raise RuntimeError(
                    'could not find the workspace root above the installed '
                    'sensor_logger package; pass log_root:=<dir> explicitly')
            log_root = str(workspace / DATA_LOG_DIRNAME)
        self.session_dir = make_session_dir(
            log_root, self.category, time.strftime('%Y%m%d-%H%M%S'))

        topics = {k: str(value(f'{k}_topic')) for k in ('imu', 'scan', 'odom')}
        self.imu_stats = ImuStats(str(value('imu_accel_unit')))
        self.imu_log = CsvLog(self.session_dir / 'imu.csv', IMU_COLUMNS)
        self.odom_log = CsvLog(self.session_dir / 'odom.csv', ODOM_COLUMNS)
        self.scan_log = ScanLog(self.session_dir / 'scan.csv')
        self._imu_zero_warned = False
        self.topics = topics
        # Every node seen publishing each recorded topic, over the whole
        # session -- a source that comes and goes still counts.
        self._publishers = {k: set() for k in topics}

        self.metadata = {
            'category': self.category,
            'category_description': spec.description,
            'protocol': spec.protocol,
            'notes': str(value('notes')),
            'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'host': socket.gethostname(),
            'source': classify_source(()),
            'source_publishers': {k: [] for k in topics},
            'topics': topics,
            'units': {'imu_accel': str(value('imu_accel_unit')),
                      'imu_gyro': str(value('imu_gyro_unit')),
                      'scan_range': 'm', 'odom_speed': 'm/s',
                      'odom_yaw_rate': 'rad/s', 'stamp': 's'},
        }
        write_json_atomic(self.session_dir / 'metadata.json', self.metadata)

        self.create_subscription(Imu, topics['imu'], self._imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, topics['scan'], self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, topics['odom'], self._odom, qos_profile_sensor_data)
        self.create_timer(float(value('status_period_sec')), self._status)
        self.create_timer(1.0, self._check_source)
        self._check_source()

        self.get_logger().info(
            f'recording category "{self.category}" to {self.session_dir}\n'
            f'  protocol: {spec.protocol}\n'
            f'  valid categories: {", ".join(CATEGORIES)}\n'
            f'  afterwards: ros2 run sensor_logger summarize_session {self.session_dir}')

    def _imu(self, msg):
        a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        self.imu_log.write((_stamp(msg.header), time.time(), a.x, a.y, a.z,
                            g.x, g.y, g.z, q.w, q.x, q.y, q.z))
        self.imu_stats.add(a.x, a.y, a.z, g.x, g.y, g.z)

    def _scan(self, msg):
        self.scan_log.write(_stamp(msg.header), time.time(), msg.angle_min,
                            msg.angle_increment, msg.range_min, msg.range_max,
                            msg.ranges)

    def _odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom_log.write((_stamp(msg.header), time.time(), p.x, p.y,
                             yaw_from_quaternion(q.w, q.x, q.y, q.z),
                             msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    def _check_source(self):
        """Who is publishing what we record? Rewrites metadata.json the
        moment the answer changes, so a hard kill still leaves it."""
        changed = False
        for key, topic in self.topics.items():
            names = {info.node_name for info in self.get_publishers_info_by_topic(topic)}
            if not names <= self._publishers[key]:
                self._publishers[key] |= names
                changed = True
        if not changed:
            return
        seen = set().union(*self._publishers.values())
        source = classify_source(seen)
        self.metadata['source'] = source
        self.metadata['source_publishers'] = {
            k: sorted(v) for k, v in self._publishers.items()}
        write_json_atomic(self.session_dir / 'metadata.json', self.metadata)
        message = f'source: {source} (publishers: {", ".join(sorted(seen))})'
        if source == SOURCE_MIXED:
            self.get_logger().warn(
                f'{message} -- more than one kind of node is publishing the '
                'recorded topics (car drivers, racerbot_sim, or something '
                'else). This recording cannot be attributed to the car or '
                'the sim.')
        else:
            self.get_logger().info(message)

    def _status(self):
        self.get_logger().info(
            f'imu {self.imu_log.rows}  scan {self.scan_log.rows} '
            f'(rejected {self.scan_log.rejected})  odom {self.odom_log.rows}')
        if (not self._imu_zero_warned and self.imu_stats.count >= 50
                and self.imu_stats.verdict() == 'all_zero'):
            self._imu_zero_warned = True
            self.get_logger().warn(
                'every IMU sample so far is exactly 0.0 -- the VESC IMU is off '
                '(docs/hardware-reference.md). Still recording, but imu.csv '
                'will contain no information.')

    def close(self):
        for log in (self.imu_log, self.odom_log, self.scan_log):
            log.close()
        self.metadata.update({
            'ended_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'counts': {'imu': self.imu_log.rows, 'odom': self.odom_log.rows,
                       'scan': self.scan_log.rows,
                       'scan_rejected': self.scan_log.rejected,
                       'scan_beams': self.scan_log.beams},
            'imu_health': self.imu_stats.as_dict(),
        })
        write_json_atomic(self.session_dir / 'metadata.json', self.metadata)


def main(args=None):
    rclpy.init(args=args)
    node = SensorLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
