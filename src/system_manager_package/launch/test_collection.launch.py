"""Test-bench launch for the full collection cycle.

Brings up every node required for a SEARCH -> SELECT -> APPROACH_OBJ ->
GRASP -> FIND_BOX -> APPROACH_BOX -> DROP pipeline, plus the Foxglove
bridge and the service-wrapper node that lets the dashboard drive
actions. The state_manager starts in IDLE so nothing moves until the
operator publishes a command on ``/robot_command`` (std_msgs/String).

Typical Foxglove operator flow:

    1. Launch this file on the robot:
           ros2 launch system_manager_package test_collection.launch.py
    2. Connect Foxglove to ws://<jetson-ip>:8765
    3. Load ``HBR_COMMAND_DASHBOARD.json`` as the layout.
    4. Publish ``"wander"`` on ``/robot_command`` to start wandering,
       ``"search"`` to start the toy search, ``"idle"`` / ``"stop"`` to halt.
    5. The dashboard's existing CallService panels now cover both TOY and
       BOX approaches, plus WskrSearch and XArm grasp, via the expanded
       wskr_foxglove_approach_bridge.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg: str, launch_file: str, launch_args: dict | None = None) -> IncludeLaunchDescription:
    launch_path = str(Path(get_package_share_directory(pkg)) / 'launch' / launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_args or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    search_aruco_id_arg = DeclareLaunchArgument(
        'search_aruco_id',
        default_value='0',
        description='ArUco marker ID used by the WSKR search behavior for BOX search.',
    )
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='TCP port for the foxglove_bridge WebSocket.',
    )

    state_manager = _include(
        'system_manager_package', 'sys_manager.launch.py',
        {'search_aruco_id': LaunchConfiguration('search_aruco_id')},
    )

    xarm = _include(
        'xarm_object_collector_package', 'xarm_object_collector_ga.launch.py',
    )

    wskr = _include('wskr', 'wskr.launch.py')

    search_supervisor_node = Node(
        package='system_manager_package',
        executable='search_supervisor.py',
        name='wskr_search_behavior',
        output='screen',
    )

    foxglove = _include(
        'utilities', 'wskr_foxglove.launch.py',
        {'port': LaunchConfiguration('foxglove_port')},
    )

    return LaunchDescription([
        search_aruco_id_arg,
        foxglove_port_arg,
        state_manager,
        xarm,
        wskr,
        search_supervisor_node,
        foxglove,
    ])
