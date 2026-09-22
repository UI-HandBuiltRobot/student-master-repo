import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from robot_config.constants import (
    GA_Z_GLOBAL_OFFSET_MM,
)


class VizPublisher(Node):
    """Publish interpolated GA trajectories to ROS2 /joint_states for RViz."""

    JOINT_NAMES = ['arm6', 'arm5', 'arm4', 'arm3', 'arm2', 'arm1']
    GRIPPER_CLOSED_RAD = 0.0
    GRIPPER_OPEN_RAD = 1.0

    def __init__(self, playback_hz=20.0):
        """Create a JointState publisher with a configurable playback rate."""
        super().__init__('ga_viz_publisher')
        self.playback_hz = float(playback_hz)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self._goal_marker = None
        self._marker_timer = self.create_timer(1.0, self._republish_goal_marker)

    def publish_goal_marker(self, goal_x_mm, goal_y_mm, goal_z_mm, frame_id='base_link'):
        """Publish a persistent sphere marker at the GA target location.

        Args:
            goal_x_mm: Target x position in millimetres.
            goal_y_mm: Target y position in millimetres.
            goal_z_mm: Target z position in millimetres.
            frame_id: RViz frame to render the marker in.
        """
        marker = Marker()
        # Use time-zero so RViz does not gate marker rendering on TF timestamp closeness.
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.header.frame_id = frame_id
        marker.ns = 'ga_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # RViz marker positions are metres; GA workspace is millimetres.
        # Apply empirical offsets so marker alignment matches observed FK in RViz.
        # Preserve the legacy X inversion so the marker appears on the correct
        # side of the base-link origin.
        # X/Y/Z are supplied directly by the planner target, with empirical
        # marker offsets layered on top as needed for RViz alignment.
        marker.pose.position.x = float(goal_x_mm) / 1000.0
        marker.pose.position.y = float(goal_y_mm) / 1000.0
        marker.pose.position.z = (float(goal_z_mm) - GA_Z_GLOBAL_OFFSET_MM + 45) / 1000.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        # Zero lifetime means the marker stays until replaced or deleted.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self._goal_marker = marker
        self._republish_goal_marker()

    def _republish_goal_marker(self):
        """Timer callback — re-stamp and republish the stored goal marker."""
        if self._goal_marker is None:
            return
        self.marker_pub.publish(self._goal_marker)

    def publish_joint_state(self, yaw_rad, arm5_rad, arm4_rad, arm3_rad, arm2_rad=0.0, arm1_rad=0.0):
        """Publish one full joint-state sample for the xArm visual model."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = [
            -float(yaw_rad),
            float(arm5_rad),
            float(arm4_rad),
            float(arm3_rad),
            float(arm2_rad),
            float(arm1_rad),
        ]
        self.joint_state_pub.publish(msg)

    def publish_gripper_state(self, arm1_rad, arm2_rad=0.0, yaw_rad=0.0, arm5_rad=0.0, arm4_rad=0.0, arm3_rad=0.0):
        """Publish a standalone gripper/wrist state update for RViz-only simulation."""
        self.publish_joint_state(
            yaw_rad=yaw_rad,
            arm5_rad=arm5_rad,
            arm4_rad=arm4_rad,
            arm3_rad=arm3_rad,
            arm2_rad=arm2_rad,
            arm1_rad=arm1_rad,
        )

    def publish_trajectory(self, traj_rad, yaw_rad=0.0, arm2_rad=0.0, arm1_rad=0.0):
        """Publish an (N, 3) radians trajectory plus a fixed base yaw to joint states."""
        trajectory = np.asarray(traj_rad, dtype=float)
        yaw_rad = float(yaw_rad)
        if trajectory.size == 0:
            return

        if trajectory.ndim != 2 or trajectory.shape[1] != 3:
            raise ValueError('Expected trajectory shape (N, 3) in radians.')

        dt = 0.0 if self.playback_hz <= 0 else 1.0 / self.playback_hz

        for frame in trajectory:
            self.publish_joint_state(
                yaw_rad=yaw_rad,
                arm5_rad=float(frame[0]),
                arm4_rad=float(frame[1]),
                arm3_rad=float(frame[2]),
                arm2_rad=arm2_rad,
                arm1_rad=arm1_rad,
            )
            rclpy.spin_once(self, timeout_sec=0.0)
            if dt > 0.0:
                time.sleep(dt)