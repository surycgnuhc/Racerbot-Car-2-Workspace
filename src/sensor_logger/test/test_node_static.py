"""
Static checks on the ROS node, the config and the launch file, read as
source text so they run with nothing sourced (the node itself imports
rclpy and is not exercised here -- see README.md, "Testing on the car").

    python3 -m pytest src/sensor_logger/test/ -v
"""
import ast
import os
import re
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sensor_logger.maneuvers import CATEGORIES  # noqa: E402
from sensor_logger.session import CAR_SENSOR_NODES, SIM_SENSOR_NODES  # noqa: E402

PKG = Path(__file__).resolve().parent.parent
NODE = PKG / 'sensor_logger' / 'sensor_logger_node.py'
CONFIG = PKG / 'config' / 'sensor_logger.yaml'
LAUNCH = PKG / 'launch' / 'sensor_logger_launch.py'
VESC_DRIVER = PKG.parent / 'f1tenth_system' / 'vesc' / 'vesc_driver' / 'src' / 'vesc_driver.cpp'


def test_package_never_publishes():
    """Subscribe-only is what makes this support tooling rather than
    driving code (docs/adding-your-own-code.md). A publisher here would
    need the full LB deadman contract."""
    banned = [r'create_publisher', r'create_client', r'call_async',
              r'set_parameters', r'\.publish\(', r'/drive', r'/ackermann_cmd',
              r'/commands/']
    sources = list((PKG / 'sensor_logger').glob('*.py')) + [LAUNCH]
    hits = [f'{p.name}:{n}: {line.strip()}'
            for p in sources
            for n, line in enumerate(p.read_text().splitlines(), 1)
            for pat in banned if re.search(pat, line)]
    assert hits == []
    assert len(sources) >= 5        # the scan actually saw the package


def _declared_parameters():
    tree = ast.parse(NODE.read_text())
    params = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'attr', None) == 'declare_parameter'):
            params[node.args[0].value] = ast.literal_eval(node.args[1])
    return params


def _config():
    return yaml.safe_load(CONFIG.read_text())['sensor_logger_node']['ros__parameters']


def test_config_and_node_declare_the_same_parameters_with_the_same_defaults():
    """Every knob in the YAML must be one the node reads, with the same
    default -- otherwise running the node without the YAML is a different
    logger from running it with."""
    declared = _declared_parameters()
    assert len(declared) >= 9
    assert _config() == declared


def test_default_category_is_in_the_catalogue():
    assert _config()['category'] in CATEGORIES


def test_launch_default_category_matches_the_config():
    source = LAUNCH.read_text()
    match = re.search(r"'category', default_value='([^']*)'", source)
    assert match and match.group(1) == _config()['category']


def test_launch_help_lists_exactly_the_catalogue():
    source = LAUNCH.read_text()
    match = re.search(r"description='Test vector: ([^']*)'\s*\n\s*'([^']*)'", source)
    listed = (match.group(1) + match.group(2)).rstrip('.').split(', ')
    assert listed == list(CATEGORIES)


def test_imu_topic_is_the_one_vesc_driver_publishes():
    """vesc_driver_node runs in the root namespace, so its relative
    'sensors/imu/raw' is /sensors/imu/raw."""
    cpp = VESC_DRIVER.read_text()
    assert re.search(r'create_publisher<Imu>\("sensors/imu/raw"', cpp)
    assert _config()['imu_topic'] == '/sensors/imu/raw'


def test_imu_units_are_the_vesc_units_not_sensor_msgs_units():
    """vesc_driver copies the VESC's values into sensor_msgs/Imu without
    converting; vesc_packet.cpp documents them as g and deg/s."""
    packet = (VESC_DRIVER.parent / 'vesc_packet.cpp').read_text()
    assert re.search(r'return acc_x_;\s*// g', packet)
    assert re.search(r'return gyr_x_;\s*// deg/s', packet)
    driver = VESC_DRIVER.read_text()
    assert 'std_imu_msg.linear_acceleration.x = imuData->acc_x();' in driver
    assert (_config()['imu_accel_unit'], _config()['imu_gyro_unit']) == ('g', 'deg/s')


STACK_LAUNCH = PKG.parent / 'f1tenth_system' / 'f1tenth_stack' / 'launch'
CAR_BRINGUPS = ['bringup_launch.py', 'sick_bringup_launch.py', 'no_lidar_bringup_launch.py']
SIM_BRINGUP = PKG.parent / 'racerbot_sim' / 'launch' / 'sim_bringup_launch.py'
# Packages whose nodes publish /scan, /odom or /sensors/imu/raw.
SENSOR_PACKAGES = {'urg_node', 'sick_scan_xd', 'vesc_driver', 'vesc_ackermann'}
NODE_BLOCK = re.compile(
    r"Node\(\s*package='([^']+)',\s*executable='([^']+)',\s*name='([^']+)'")


def _sensor_node_names(launch_file):
    """(package, name) of every sensor-publishing Node in a launch file.
    ackermann_to_vesc_node is in vesc_ackermann but publishes commands, not
    sensor data, so it is left out."""
    return {(pkg, name) for pkg, exe, name in NODE_BLOCK.findall(launch_file.read_text())
            if pkg in SENSOR_PACKAGES and exe != 'ackermann_to_vesc_node'}


@pytest.mark.parametrize('launch', CAR_BRINGUPS)
def test_every_car_sensor_node_is_recognised_as_car(launch):
    """A rename in a bringup would otherwise turn real-car sessions into
    'unknown' or 'mixed' without anyone noticing."""
    found = _sensor_node_names(STACK_LAUNCH / launch)
    assert len(found) >= 2          # the regex actually matched this file
    assert {name for _, name in found} <= CAR_SENSOR_NODES


def test_every_car_sensor_name_is_one_a_bringup_actually_uses():
    used = set().union(*({n for _, n in _sensor_node_names(STACK_LAUNCH / f)}
                         for f in CAR_BRINGUPS))
    assert used == CAR_SENSOR_NODES


def test_the_sim_node_is_the_one_that_publishes_scan_and_odom():
    sim = SIM_BRINGUP.read_text()
    assert re.search(r"executable='gym_bridge_node',\s*name='gym_bridge_node'", sim)
    bridge = (PKG.parent / 'racerbot_sim' / 'racerbot_sim' / 'gym_bridge_node.py').read_text()
    assert "create_publisher(LaserScan, str(value('scan_topic'))" in bridge
    assert "create_publisher(Odometry, str(value('odom_topic'))" in bridge
    assert SIM_SENSOR_NODES == {'gym_bridge_node'}
