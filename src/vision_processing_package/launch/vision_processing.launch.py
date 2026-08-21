from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from robot_config.constants import (
    XYZ_CONFIG_FILENAME, 
    BBOX_TO_XYZ_MODE,)


def generate_launch_description():
    """Build launch graph for camera, vision inference, and bbox-to-xyz service nodes."""

    bbox_to_xyz_mode_arg = DeclareLaunchArgument(
        'bbox_to_xyz_mode',
        default_value=BBOX_TO_XYZ_MODE,
    )
    
    # calibration path for bbox_to_xyz_2d_node (only resolved when node starts)
    calibration_path_arg = DeclareLaunchArgument(
        'calibration_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('vision_processing_package'),
            'config',
            XYZ_CONFIG_FILENAME,
        ]),
        description='Path to camera calibration JSON file (only used for 2d mode)',
    )

    gstreamer_camera_node = Node(
        package='vision_processing_package',
        executable='gst_cam_node',
        name='gstreamer_camera',
        output='screen',
    )

    obj_det_node = Node(
        package='vision_processing_package',
        executable='process_object_vision',
        name='process_object_vision',
        output='both',
    )

    object_selection_node = Node(
        package='vision_processing_package',
        executable='object_selection',
        name='object_selection_node',
        output='both',
    )

    bbox_to_xyz_node = Node(
        package='vision_processing_package',
        executable='bbox_to_xyz_service',
        name='bbox_to_xyz_node',
        output='both',
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('bbox_to_xyz_mode'),
            "'.lower() == 'simple'",
        ])),
    )

    bbox_to_xyz_2d_node = Node(
        package='vision_processing_package',
        executable='bbox_to_xyz_service_2D',
        name='bbox_to_xyz_node',
        output='both',
        parameters=[{
            'calibration_path': LaunchConfiguration('calibration_path'),
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('bbox_to_xyz_mode'),
            "'.lower() == '2d'",
        ])),
    )

    return LaunchDescription([
        bbox_to_xyz_mode_arg,
        calibration_path_arg,
        gstreamer_camera_node,
        obj_det_node,
        object_selection_node,
        bbox_to_xyz_node,
        bbox_to_xyz_2d_node,
    ])
