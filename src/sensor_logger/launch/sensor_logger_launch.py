"""Record IMU + LiDAR + odometry into <workspace>/data_logs/<category>/<time>/.

Run it in its own terminal ALONGSIDE bringup and a control layer -- like
race_diagnostics, it is a subscribe-only layer on top of the two-tier
launch pattern (docs/architecture.md) and starts nothing that can move the
car:

    ros2 launch sensor_logger sensor_logger_launch.py category:=turn_left
    ros2 launch sensor_logger sensor_logger_launch.py category:=stop notes:='carpet, 1 m/s'

Ctrl-C ends the session. Then:

    ros2 run sensor_logger summarize_session <the folder it printed>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('sensor_logger'),
                          'config', 'sensor_logger.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'category', default_value='free_drive',
            description='Test vector: idle, start, stop, straight, turn_left, '
                        'turn_right, reverse, free_drive.'),
        DeclareLaunchArgument(
            'notes', default_value='',
            description='Free text stored in metadata.json (surface, speed, battery...).'),
        DeclareLaunchArgument(
            'log_root', default_value='',
            description='Parent folder. Empty = <workspace>/data_logs.'),
        Node(
            package='sensor_logger',
            executable='sensor_logger_node',
            name='sensor_logger_node',
            output='screen',
            # Overrides AFTER the YAML, so the command line wins. Forced to
            # str: launch_ros YAML-parses substitutions, which would turn
            # notes:='' into None and notes:=1 into an int.
            parameters=[config, {
                name: ParameterValue(LaunchConfiguration(name), value_type=str)
                for name in ('category', 'notes', 'log_root')
            }],
        ),
    ])
