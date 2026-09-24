#!/usr/bin/env python3
"""WSKR Search Behavior — Action Server for wandering while searching.

Action server that accepts a goal specifying target type (TOY or BOX),
then wanders around a room while searching for the target.

For TOY:
    - Watches the YOLO detection stream (vision/yolo/detections) during LOOK cycles
    - Returns the full detection frame when any detection passes the confidence gate
For BOX (ArUco marker):
    - Watches WSKR/aruco_markers (published by the approach action server)
    - Returns detection when target marker is found
Both modes use WSKR safety features (floor mask monitoring, whisker monitoring).

Returns:
    - SUCCESS: when a valid detection matching the target type is found.
    - ABORT: on timeout or other failure.

Action Interface:
    WSKR/search_behavior (WskrSearch)
        Goal: target_type (TOY or BOX), target_id, timeout_sec
        Result: success, detected_object (ImgDetectionData)

Topics:
    subscribes  WSKR/floor_mask                 — floor mask from wskr_floor_node
    subscribes  WSKR/aruco_markers              — detected markers from approach_action_server
    subscribes  WSKR/whisker_lengths            — safety check
    subscribes  vision/yolo/detections          — streaming YOLO tracker output
    publishes   WSKR/heading_to_target          — desired heading for the WSKR autopilot

Note on threading model (rclpy Humble):
    MultiThreadedExecutor runs async action execute-callbacks in a thread-pool
    that does NOT own an asyncio event loop, so asyncio.sleep / asyncio.wrap_future
    cannot be used here.  All sleeps are plain time.sleep(); the ReentrantCallbackGroup
    ensures subscription callbacks (image, whiskers) are dispatched on separate
    threads and are therefore never starved by these sleeps.
"""
import math
import random
import threading
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray

from robot_interfaces.action import WskrSearch
from robot_interfaces.msg import ImgDetectionData

from robot_config.robot_config.constants import (
    SELECTION_CLASS_PRIORITIES,
    SEARCH_ARUCO_STALE_TIMEOUT_SEC,
    SEARCH_CONFIDENCE_THRESHOLD,
    SEARCH_FLOOR_OBSTACLE_RATIO,
    SEARCH_HEADING_CHANGE_PERIOD_SEC,
    SEARCH_LOOK_DURATION_SEC,
    SEARCH_MAX_HEADING_ANGLE_DEG,
    SEARCH_POLL_INTERVAL_SEC,
    SEARCH_TIMEOUT_SEC,
    SEARCH_WANDER_SPEED_MPS,
    SEARCH_ARUCO_ID,
)


IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_POLL_INTERVAL = SEARCH_POLL_INTERVAL_SEC

# Floor obstacle threshold: if more than this ratio of the bottom-center
# region is non-floor, treat it as an obstacle ahead.
_FLOOR_OBSTACLE_RATIO = SEARCH_FLOOR_OBSTACLE_RATIO


class SearchBehavior(Node):
    """Wander + search behavior action server with WSKR safety."""

    TARGET_TOY = 0
    TARGET_BOX = 1

    def __init__(self) -> None:
        super().__init__('wskr_search_behavior')

        # Parameters
        self.declare_parameter('wander_speed_m_s', SEARCH_WANDER_SPEED_MPS)
        self.declare_parameter('look_duration_sec', SEARCH_LOOK_DURATION_SEC)
        self.declare_parameter('confidence_threshold', SEARCH_CONFIDENCE_THRESHOLD)
        self.declare_parameter('aruco_id', SEARCH_ARUCO_ID)
        self.declare_parameter('heading_change_period_sec', SEARCH_HEADING_CHANGE_PERIOD_SEC)
        self.declare_parameter('max_heading_angle', SEARCH_MAX_HEADING_ANGLE_DEG)

        self.wander_speed            = float(self.get_parameter('wander_speed_m_s').value)
        self.look_duration           = float(self.get_parameter('look_duration_sec').value)
        self.conf_threshold          = float(self.get_parameter('confidence_threshold').value)
        self.aruco_id                = int(self.get_parameter('aruco_id').value)
        self.heading_change_period   = float(self.get_parameter('heading_change_period_sec').value)
        self.max_heading_angle       = float(self.get_parameter('max_heading_angle').value)
        self.allowed_toy_classes     = {str(c) for c in SELECTION_CLASS_PRIORITIES}

        # State
        self.lock = threading.Lock()
        self.latest_whiskers: Optional[np.ndarray] = None
        self.latest_detections: Optional[ImgDetectionData] = None
        self.latest_detections_t: Optional[float] = None
        self.latest_floor_mask: Optional[np.ndarray] = None
        self.latest_aruco_markers: Optional[Float32MultiArray] = None
        self.latest_aruco_t: Optional[float] = None
        self.current_heading_deg = 0.0
        self.last_heading_change = time.time()

        cb_group = ReentrantCallbackGroup()

        self.floor_mask_sub = self.create_subscription(
            Image,
            'WSKR/floor_mask',
            self._on_floor_mask,
            IMAGE_QOS,
            callback_group=cb_group,
        )
        self.aruco_sub = self.create_subscription(
            Float32MultiArray,
            'WSKR/aruco_markers',
            self._on_aruco_markers,
            10,
            callback_group=cb_group,
        )
        self.whisker_sub = self.create_subscription(
            Float32MultiArray,
            'WSKR/whisker_lengths',
            self._on_whiskers,
            10,
            callback_group=cb_group,
        )
        self.detections_sub = self.create_subscription(
            ImgDetectionData,
            'vision/yolo/detections',
            self._on_detections,
            IMAGE_QOS,
            callback_group=cb_group,
        )

        self.heading_pub = self.create_publisher(Float32, 'WSKR/heading_to_target', 10)
        autopilot_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.autopilot_enable_pub = self.create_publisher(Bool, 'WSKR/autopilot/enable', autopilot_qos)

        self._action_server = ActionServer(
            self,
            WskrSearch,
            'WSKR/search_behavior',
            execute_callback=self._execute_search,
            cancel_callback=self._handle_cancel,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'Search behavior action server ready '
            f'(confidence_threshold={self.conf_threshold:.2f}).'
        )

    def _reset_goal_caches(self) -> None:
        """Drop stale per-goal sensor snapshots that can cause false immediate success."""
        with self.lock:
            self.latest_detections = None
            self.latest_detections_t = None
            self.latest_aruco_markers = None
            self.latest_aruco_t = None

    def _has_selectable_toy_detection(self, msg: ImgDetectionData) -> bool:
        """Return True only if a detection passes confidence and class filters."""
        n = len(msg.x)
        for i in range(n):
            conf = float(msg.confidence[i]) if i < len(msg.confidence) else 0.0
            if conf < self.conf_threshold:
                continue
            cls = str(msg.class_name[i]) if i < len(msg.class_name) else ''
            if cls in self.allowed_toy_classes:
                return True
        return False

    # ------------------------------------------------------------------ #
    # Subscription callbacks                                               #
    # ------------------------------------------------------------------ #

    def _on_floor_mask(self, msg: Image) -> None:
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        with self.lock:
            self.latest_floor_mask = mask

    def _on_aruco_markers(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.latest_aruco_markers = msg
            self.latest_aruco_t = time.time()

    def _on_whiskers(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape[0] == 11:
            with self.lock:
                self.latest_whiskers = data

    def _on_detections(self, msg: ImgDetectionData) -> None:
        with self.lock:
            self.latest_detections = msg
            self.latest_detections_t = time.time()

    # ------------------------------------------------------------------ #
    # Robot control helpers                                                #
    # ------------------------------------------------------------------ #

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Search behavior goal cancellation received.')
        return CancelResponse.ACCEPT

    def _publish_heading(self, heading_deg: float) -> None:
        if not rclpy.ok():
            return
        msg = Float32()
        msg.data = float(heading_deg)
        try:
            self.heading_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'Unable to publish heading: {exc}')

    def _enable_autopilot(self, enabled: bool) -> None:
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = bool(enabled)
        try:
            self.autopilot_enable_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'Unable to publish autopilot enable: {exc}')

    def _stop_robot(self) -> None:
        if not rclpy.ok():
            return
        self._publish_heading(0.0)
        self._enable_autopilot(False)

    # ------------------------------------------------------------------ #
    # Heading / wander logic                                               #
    # ------------------------------------------------------------------ #

    def _select_search_heading(self) -> None:
        with self.lock:
            whiskers = self.latest_whiskers

        if whiskers is None or len(whiskers) != 11:
            return

        left   = float(np.mean(whiskers[:5]))
        right  = float(np.mean(whiskers[6:]))
        center = float(np.mean(whiskers[4:7]))
        goal = 0.0

        if center < 200.0:
            goal = 45.0 if left > right else -45.0
        elif left < 200.0:
            goal = -30.0
        elif right < 200.0:
            goal = 30.0

        self.current_heading_deg = max(
            -self.max_heading_angle, min(self.max_heading_angle, goal)
        )
        self.last_heading_change = time.time()

    def _update_search_heading(self) -> None:
        now = time.time()
        if now - self.last_heading_change <= self.heading_change_period:
            return

        self._select_search_heading()

        if math.isclose(self.current_heading_deg, 0.0, abs_tol=1e-3):
            self.current_heading_deg = random.choice([-45.0, -30.0, 0.0, 30.0, 45.0])
            self.get_logger().info(
                f'WANDER: selected random heading {self.current_heading_deg:.1f}° '
                '(initial zero heading)'
            )
        else:
            self.current_heading_deg += random.choice([-15.0, 0.0, 15.0])
            self.get_logger().info(
                f'WANDER: perturbed heading to {self.current_heading_deg:.1f}°'
            )

        self.current_heading_deg = max(
            -self.max_heading_angle, min(self.max_heading_angle, self.current_heading_deg)
        )
        self.last_heading_change = now

    # ------------------------------------------------------------------ #
    # Vision helpers (subscribe to existing topics, no duplicate compute)  #
    # ------------------------------------------------------------------ #

    def _check_floor_obstacle(self) -> bool:
        """Check the latest floor mask for obstacles in the bottom-center region."""
        with self.lock:
            mask = self.latest_floor_mask

        if mask is None:
            return False

        h, w = mask.shape
        bottom_center = mask[h // 2:, w // 4: 3 * w // 4]
        non_floor_ratio = np.sum(bottom_center == 0) / bottom_center.size
        return non_floor_ratio > _FLOOR_OBSTACLE_RATIO

    def _check_aruco_target(self, target_id: int) -> Optional[Tuple[int, Tuple[float, float, float, float]]]:
        """Check cached WSKR/aruco_markers for the target marker.

        Returns (marker_id, (cx_norm, cy_norm, w_norm, h_norm)) or None.
        The approach_action_server publishes 9 floats per marker:
        [id, x0/fw, y0/fw, x1/fw, y1/fw, x2/fw, y2/fw, x3/fw, y3/fw]
        where coords are width-normalized corner positions.
        """
        with self.lock:
            msg = self.latest_aruco_markers
            msg_t = self.latest_aruco_t

        if msg is None or msg_t is None:
            return None
        # Stale check: ignore markers older than configured timeout.
        if time.time() - msg_t > SEARCH_ARUCO_STALE_TIMEOUT_SEC:
            return None

        data = msg.data
        if len(data) < 9:
            return None

        for i in range(0, len(data), 9):
            marker_id = int(data[i])
            if marker_id != target_id:
                continue
            # Extract corner coords (width-normalized)
            xs = [data[i + 1], data[i + 3], data[i + 5], data[i + 7]]
            ys = [data[i + 2], data[i + 4], data[i + 6], data[i + 8]]
            x_min = min(xs)
            y_min = min(ys)
            w = max(xs) - x_min
            h = max(ys) - y_min
            return (marker_id, (x_min, y_min, w, h))

        return None

    def _box_detection_to_img_data(
        self,
        marker_id: int,
        bbox_norm: Tuple[float, float, float, float],
        frame_width: int = 960,
    ) -> ImgDetectionData:
        """Convert width-normalized ArUco bbox to ImgDetectionData."""
        x_min, y_min, w, h = bbox_norm
        obj = ImgDetectionData()
        obj.image_width   = frame_width
        obj.image_height  = frame_width  # approximate; only width is known from normalization
        obj.detection_ids = [str(marker_id)]
        obj.x             = [float((x_min + w / 2) * frame_width)]
        obj.y             = [float((y_min + h / 2) * frame_width)]
        obj.width         = [float(w * frame_width)]
        obj.height        = [float(h * frame_width)]
        obj.confidence    = [1.0]
        obj.class_name    = ['box']
        obj.distance      = [0.0]
        obj.aspect_ratio  = [float(w) / float(h) if h > 0 else 1.0]
        return obj

    # ------------------------------------------------------------------ #
    # Action execute callback                                              #
    # ------------------------------------------------------------------ #

    async def _execute_search(self, goal_handle) -> WskrSearch.Result:
        """Entry point called by rclpy action server (runs on executor thread)."""
        goal = goal_handle.request
        target_type = goal.target_type
        target_id   = int(goal.target_id) if hasattr(goal, 'target_id') else 0
        if target_type == self.TARGET_BOX and target_id <= 0:
            target_id = self.aruco_id
        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec > 0 else SEARCH_TIMEOUT_SEC

        target_name = 'TOY' if target_type == self.TARGET_TOY else 'BOX'
        self.get_logger().info(
            f'Search behavior started '
            f'(target={target_name}, target_id={target_id}, timeout={timeout_sec}s)'
        )

        self._enable_autopilot(True)
        self.current_heading_deg = 0.0
        self.last_heading_change = time.time()

        start_time = time.time()
        self._reset_goal_caches()
        detected_object = None

        try:
            if target_type == self.TARGET_TOY:
                detected_object = self._search_toy(goal_handle, start_time, timeout_sec)
            else:
                detected_object = self._search_box(goal_handle, start_time, timeout_sec, target_id)
        finally:
            self._stop_robot()

        result = WskrSearch.Result()
        if detected_object is not None:
            result.success = True
            result.detected_object = detected_object
            goal_handle.succeed()
            self.get_logger().info('Search behavior succeeded with detection.')
        else:
            result.success = False
            goal_handle.abort()
            self.get_logger().warn('Search behavior aborted (no detection found).')

        return result

    # ------------------------------------------------------------------ #
    # Search loops (synchronous, blocking — safe on executor worker thread)
    # ------------------------------------------------------------------ #

    def _search_toy(
        self, goal_handle, start_time: float, timeout_sec: float
    ) -> Optional[ImgDetectionData]:
        """Wander while watching the YOLO detection stream.

        Heading is re-published every poll tick so the autopilot watchdog
        is never starved during a LOOK window. The first streaming frame
        with any detection above ``confidence_threshold`` is returned
        in full — downstream ``object_selection`` picks the winner by
        class priority + y-position.
        """
        last_seen_t: Optional[float] = start_time

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Toy search cancelled.')
                goal_handle.canceled()
                return None

            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(
                    f'Toy search timeout after {time.time() - start_time:.1f}s'
                )
                return None

            self._update_search_heading()
            self._publish_heading(self.current_heading_deg)

            look_end_time = time.time() + self.look_duration

            while time.time() < look_end_time:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Toy search cancelled during LOOK.')
                    goal_handle.canceled()
                    return None

                if time.time() - start_time > timeout_sec:
                    return None

                self._publish_heading(self.current_heading_deg)

                with self.lock:
                    msg = self.latest_detections
                    msg_t = self.latest_detections_t

                if (
                    msg is not None
                    and msg_t is not None
                    and (last_seen_t is None or msg_t > last_seen_t)
                ):
                    last_seen_t = msg_t
                    if self._has_selectable_toy_detection(msg):
                        self.get_logger().info(
                            f'Toy detections: n={len(msg.x)} '
                            f'classes={list(msg.class_name)} '
                            f'track_ids={list(msg.detection_ids)}'
                        )
                        return msg

                time.sleep(_POLL_INTERVAL)

        return None

    def _search_box(
        self, goal_handle, start_time: float, timeout_sec: float, target_id: int
    ) -> Optional[ImgDetectionData]:
        """Watch WSKR/aruco_markers and WSKR/floor_mask while wandering."""
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Box search cancelled.')
                goal_handle.canceled()
                return None

            if time.time() - start_time > timeout_sec:
                self.get_logger().warn(
                    f'Box search timeout after {time.time() - start_time:.1f}s'
                )
                return None

            self._update_search_heading()
            self._publish_heading(self.current_heading_deg)

            if self._check_floor_obstacle():
                self.get_logger().info('Obstacle detected ahead, adjusting heading.')
                self._select_search_heading()
                self._publish_heading(self.current_heading_deg)
                time.sleep(0.5)

            aruco_result = self._check_aruco_target(target_id)
            if aruco_result is not None:
                marker_id, bbox_norm = aruco_result
                detected_object = self._box_detection_to_img_data(marker_id, bbox_norm)
                self.get_logger().info(f'Box detected: ArUco ID={marker_id}')
                return detected_object

            time.sleep(_POLL_INTERVAL)

        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SearchBehavior()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
