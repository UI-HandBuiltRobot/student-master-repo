"""Launch the foxglove_bridge WebSocket + the wskr_web_helper model lister.

Run alongside wskr.launch.py when you want the web dashboard:

    ros2 launch utilities wskr_foxglove.launch.py

Viewers connect to ws://<jetson-ip>:8765 from Foxglove Studio (desktop,
self-hosted container, or app.foxglove.dev).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='TCP port for the foxglove_bridge WebSocket.',
    )
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Bind address for the bridge. 0.0.0.0 accepts LAN clients.',
    )
    # Image feeds into the dashboard are throttled independently of the
    # underlying publish rate — vision nodes still get the full stream.
    # Camera and overlay have separate knobs because they carry different
    # information densities: the overlay needs to be responsive enough
    # that whisker rays animate visibly, the camera can be slower.
    camera_rate_arg = DeclareLaunchArgument(
        'bridge_camera_rate_hz',
        default_value='2.0',
        description=(
            'Rate at which /camera1/image_raw/compressed is forwarded '
            'to foxglove_bridge (topic: /camera1/throttled/compressed).'
        ),
    )
    overlay_rate_arg = DeclareLaunchArgument(
        'bridge_overlay_rate_hz',
        default_value='5.0',
        description=(
            'Rate at which /wskr_overlay/compressed is forwarded to '
            'foxglove_bridge (topic: /wskr_overlay/throttled/compressed). '
            'Higher than the camera rate so whisker-ray animation stays '
            'fluid; the overlay composite is still cheap.'
        ),
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        camera_rate_arg,
        overlay_rate_arg,
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                # Preserve the durability of latched publishers so the
                # dashboard picks up the last value on connect.
                'use_compression': False,
                # Low-latency live streaming: tell the bridge's ROS
                # subscribers to keep only the newest message per topic
                # instead of the default depth=25. With depth=1 a slow or
                # bursty network drops stale frames instead of buffering
                # them on the Jetson and burst-draining later.
                'min_qos_depth': 1,
                'max_qos_depth': 1,
                # Cap the per-client WebSocket send buffer — if the client
                # can't keep up, the bridge disconnects it rather than
                # accumulating ever-more-delayed frames. 2 MB is plenty
                # for several JPEG images worth of slack but nowhere near
                # the 10 MB default that masks network misbehaviour.
                'send_buffer_limit': 2_000_000,
            }],
        ),
        Node(
            package='utilities',
            executable='wskr_web_helper',
            name='wskr_web_helper',
            output='screen',
        ),
        Node(
            package='utilities',
            executable='wskr_foxglove_approach_bridge',
            name='wskr_foxglove_approach_bridge',
            output='screen',
        ),
        # topic_tools throttle: republish the heavy image topics at a
        # lower rate just for the dashboard. Vision nodes keep consuming
        # the originals. The throttle uses positional args:
        #   throttle messages <in_topic> <msgs_per_sec> <out_topic>
        Node(
            package='topic_tools',
            executable='throttle',
            name='camera_throttle',
            arguments=[
                'messages',
                '/camera1/image_raw/compressed',
                LaunchConfiguration('bridge_camera_rate_hz'),
                '/camera1/throttled/compressed',
            ],
            output='screen',
        ),
        Node(
            package='topic_tools',
            executable='throttle',
            name='overlay_throttle',
            arguments=[
                'messages',
                '/wskr_overlay/compressed',
                LaunchConfiguration('bridge_overlay_rate_hz'),
                '/wskr_overlay/throttled/compressed',
            ],
            output='screen',
        ),
        # ArUco marker detection is handled by wskr_approach_action, which
        # publishes WSKR/aruco_markers whenever someone subscribes — the
        # Foxglove panel reads from that topic directly. No separate
        # dashboard-side detector needed.
    ])
