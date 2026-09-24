from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg: str, launch_file: str, launch_args: dict | None = None) -> IncludeLaunchDescription:
    launch_path = str(Path(get_package_share_directory(pkg)) / 'launch' / launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_args or {}).items(),
    )


def generate_launch_description():

    search_aruco_id_arg = DeclareLaunchArgument(
        'search_aruco_id',
        default_value='0',
        description='ArUco marker ID used by WSKR search behavior for box search.',
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
        executable='search_supervisor',
        name='wskr_search_behavior',
        output='screen',
        parameters=[{
            'wander_speed_m_s': 0.1,
            'look_duration_sec': 5.0,
            'confidence_threshold': 0.5,
            'aruco_id': LaunchConfiguration('search_aruco_id'),
            'max_heading_angle': 45.0,
        }],
    )

    return LaunchDescription([
        search_aruco_id_arg,
        state_manager,
        xarm,
        wskr,
        search_supervisor_node,
    ])
