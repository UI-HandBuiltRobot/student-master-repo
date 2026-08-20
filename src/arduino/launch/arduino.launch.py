"""Launch the mecanum serial bridge with parameters from config/serial.yaml."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    bridge_node = Node(
        package='arduino',
        executable='mecanum_serial_bridge',
        name='mecanum_serial_bridge',
        output='screen',
    )

    return LaunchDescription([bridge_node])
