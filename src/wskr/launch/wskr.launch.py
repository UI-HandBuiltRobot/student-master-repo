from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arduino_launch = str(
        Path(get_package_share_directory('arduino')) / 'launch' / 'arduino.launch.py'
    )
    vision_launch = str(
        Path(get_package_share_directory('vision_processing_package'))
        / 'launch' / 'vision_processing.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
        ),
        Node(
            package='wskr',
            executable='wskr_floor',
            name='wskr_floor',
            output='screen',
        ),
        Node(
            package='wskr',
            executable='wskr_range',
            name='wskr_range',
            output='screen',
        ),
        Node(
            package='wskr',
            executable='wskr_approach_action',
            name='wskr_approach_action',
            output='screen',
        ),
        Node(
            package='wskr',
            executable='wskr_dead_reckoning',
            name='wskr_dead_reckoning',
            output='screen',
        ),
        Node(
            package='wskr',
            executable='wskr_autopilot',
            name='wskr_autopilot',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arduino_launch),
        ),
    ])
