from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sys_manager = Node(
        package='system_manager_package',
        executable='state_manager',
        name='state_manager_node',
        output='both',
    )

    return LaunchDescription([
        sys_manager,
    ])

