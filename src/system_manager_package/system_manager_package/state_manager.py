#!/usr/bin/env python3
"""Finite-state machine for the robot collection pipeline.

State flow (continuous collection):

    IDLE ──(command)──► SEARCH ──► SELECT ──► APPROACH_OBJ ──► GRASP
                           ▲                                     │
                           │                                     ▼
                           │                                  FIND_BOX
                           │                                     │
                           │                                     ▼
                        SEARCH ◄── DROP ◄── APPROACH_BOX ◄───────┘


    STOPPED / ERROR cancel all in-flight actions and ignore further
    commands (except "idle" / "stop" to escape STOPPED).

Movement is indirect: state handlers send action goals to the
search-behavior and approach action servers, which publish headings
that the autopilot MLP converts into cmd_vel.

Control interface:
    Subscribe  /robot_command  (std_msgs/String)  — lowercase state name
    Publish    /robot_state    (std_msgs/String)  — uppercase state name
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from enum import Enum
import threading
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from robot_interfaces.srv import SelectObject  # type: ignore
from robot_interfaces.action import ApproachObject, XArm, WskrSearch  # type: ignore

from robot_config.robot_config.constants import (
    SELECTION_CLASS_PRIORITIES,
    SM_BOX_ARUCO_ID,
    SM_DELAY_APPROACH_BOX,
    SM_DELAY_APPROACH_OBJ,
    SM_DELAY_DROP,
    SM_DELAY_FIND_BOX,
    SM_DELAY_GRASP,
    SM_DELAY_SEARCH,
    SM_DELAY_SELECT,
    SM_MAX_GRASP_RETRIES,
    SEARCH_TIMEOUT_SEC,
)


class RobotState(Enum):
    IDLE = 0
    SEARCH = 1
    SELECT = 2
    APPROACH_OBJ = 3
    GRASP = 4
    FIND_BOX = 5
    APPROACH_BOX = 6
    DROP = 7
    STOPPED = 8
    ERROR = 9

_COMMAND_MAP = {
    "idle":         RobotState.IDLE,
    "search":       RobotState.SEARCH,
    "select":       RobotState.SELECT,
    "approach_obj": RobotState.APPROACH_OBJ,
    "grasp":        RobotState.GRASP,
    "find_box":     RobotState.FIND_BOX,
    "approach_box": RobotState.APPROACH_BOX,
    "drop":         RobotState.DROP,
    "stopped":      RobotState.STOPPED,
    "stop":         RobotState.STOPPED,
}


class StateManagerNode(Node):
    """Orchestrates the collection pipeline by dispatching ROS actions/services
    and routing their results to the next FSM state."""

    def __init__(self):
        super().__init__('state_manager_node')

        # ── FSM bookkeeping ─────────────────────────────────────────
        self._state = RobotState.IDLE
        self._lock = threading.Lock()
        self._timer = None
        self._goal_handles: list = []

        self.selected_object = None
        self._search_target_type = None
        self._search_target_id = 0
        self._grasp_retries = 0
        self._max_grasp_retries = SM_MAX_GRASP_RETRIES
        self._allowed_toy_classes = {str(c) for c in SELECTION_CLASS_PRIORITIES}

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('search_aruco_id', SM_BOX_ARUCO_ID)
        self._box_aruco_id = int(self.get_parameter('search_aruco_id').value)

        # ── ROS interfaces ──────────────────────────────────────────
        self._state_pub = self.create_publisher(String, 'robot_state', 10)
        self._stop_pub = self.create_publisher(Empty, 'WSKR/stop', 1)
        self.create_subscription(String, 'robot_command', self._on_command, 10)

        self._select_cli = self.create_client(SelectObject, 'select_object_service')
        self._gripper_cli = self.create_client(Trigger, 'open_gripper_service')

        self._search_ac = ActionClient(self, WskrSearch, 'WSKR/search_behavior')
        self._approach_ac = ActionClient(self, ApproachObject, 'WSKR/approach_object')
        self._grasp_ac = ActionClient(self, XArm, 'xarm_grasp_action')

        # ── Dispatch table: state → (delay_sec, handler) ───────────
        # Each handler fires once after delay_sec (one-shot timer).
        self._dispatch = {
            RobotState.SEARCH:       (SM_DELAY_SEARCH, self._do_search),
            RobotState.SELECT:       (SM_DELAY_SELECT, self._do_select),
            RobotState.APPROACH_OBJ: (SM_DELAY_APPROACH_OBJ, self._do_approach_obj),
            RobotState.GRASP:        (SM_DELAY_GRASP, self._do_grasp),
            RobotState.FIND_BOX:     (SM_DELAY_FIND_BOX, self._do_find_box),
            RobotState.APPROACH_BOX: (SM_DELAY_APPROACH_BOX, self._do_approach_box),
            RobotState.DROP:         (SM_DELAY_DROP, self._do_drop),
        }

        self.get_logger().info(
            'State manager ready in IDLE. Publish to /robot_command to begin.'
        )
        self._transition(RobotState.IDLE)

    # ================================================================
    #  FSM core
    # ================================================================

    def _get_state(self) -> RobotState:
        with self._lock:
            return self._state

    def _transition(self, state: RobotState):
        """Cancel any pending work, move to *state*, publish it, and
        schedule the state's handler (if any).

        STOPPED / ERROR additionally cancel all in-flight action goals.
        IDLE just publishes and waits.
        """
        # Cancel pending one-shot timer
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        with self._lock:
            self._state = state
        self.get_logger().info(f"STATE -> {state.name}")

        msg = String()
        msg.data = state.name
        self._state_pub.publish(msg)

        if state in (RobotState.STOPPED, RobotState.ERROR):
            self._stop_pub.publish(Empty())
            self._cancel_all_goals()
            return

        entry = self._dispatch.get(state)
        if entry is not None:
            delay, handler = entry
            self._timer = self.create_timer(delay, handler)

    def _is_halted(self) -> bool:
        """True when the FSM should swallow late-arriving results."""
        return self._get_state() in (RobotState.STOPPED, RobotState.ERROR)

    # ── Goal tracking / cancellation ────────────────────────────────

    def _track(self, goal_handle):
        self._goal_handles.append(goal_handle)

    def _cancel_all_goals(self):
        """Cancel every tracked action goal (called on STOPPED/ERROR)."""
        for gh in self._goal_handles:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Goal cancel failed: {e}")
        self._goal_handles.clear()

    # ── Availability guards ─────────────────────────────────────────

    def _require_service(self, client, name: str) -> bool:
        if not client.service_is_ready():
            self.get_logger().error(f"{name} not available -> ERROR")
            self._transition(RobotState.ERROR)
            return False
        return True

    def _require_action(self, client, name: str) -> bool:
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{name} not available -> ERROR")
            self._transition(RobotState.ERROR)
            return False
        return True

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    # ================================================================
    #  Command interface  (/robot_command subscriber)
    # ================================================================

    def _on_command(self, msg: String):
        """STOPPED is latching: only idle/stop/stopped are accepted."""
        cmd = msg.data.strip().lower()

        if (self._get_state() == RobotState.STOPPED
                and cmd not in ("idle", "stop", "stopped")):
            self.get_logger().info(f"STOPPED — ignoring '{cmd}'")
            return

        state = _COMMAND_MAP.get(cmd)
        if state is not None:
            self._transition(state)
        else:
            self.get_logger().warn(f"Unknown command: '{cmd}'")

    # ================================================================
    #  SEARCH / FIND_BOX 
    #
    #  All three use the WskrSearch action with shared callbacks.
    #  _search_target_type tracks which variant is active so results
    #  route correctly (TOY -> SELECT, BOX -> APPROACH_BOX).
    # ================================================================

    def _do_search(self):
        """SEARCH: wander while looking for a toy (YOLO detection)."""
        self._cancel_timer()
        if not self._require_action(self._search_ac, 'WSKR/search_behavior'):
            return
        self._send_search_goal(WskrSearch.Goal.TARGET_TOY, 0)

    def _do_find_box(self):
        """FIND_BOX: wander while looking for the drop-box ArUco marker."""
        self._cancel_timer()
        if not self._require_action(self._search_ac, 'WSKR/search_behavior'):
            return
        self._send_search_goal(WskrSearch.Goal.TARGET_BOX, self._box_aruco_id)

    def _send_search_goal(self, target_type: int, target_id: int):
        self._search_target_type = target_type
        self._search_target_id = int(target_id)

        goal = WskrSearch.Goal()
        goal.target_type = int(target_type)
        goal.target_id = int(target_id)
        goal.timeout_sec = SEARCH_TIMEOUT_SEC

        future = self._search_ac.send_goal_async(
            goal, feedback_callback=self._on_search_fb,
        )
        future.add_done_callback(self._on_search_accepted)

    def _on_search_fb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(
            f"SEARCH fb: phase={fb.current_phase}, "
            f"elapsed={fb.elapsed_sec:.1f}s, sampled={fb.detections_sampled}"
        )

    def _on_search_accepted(self, future):
        goal_handle = future.result()
        self._track(goal_handle)
        if not goal_handle.accepted:
            self.get_logger().warn("Search goal rejected — retrying")
            self._retry_search()
            return
        goal_handle.get_result_async().add_done_callback(self._on_search_result)

    def _on_search_result(self, future):
        """Route: TOY found -> SELECT, BOX found -> APPROACH_BOX,
        nothing -> retry same search type."""
        if self._is_halted():
            return
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Search failed: {e}")
            self._retry_search()
            return

        found = bool(result.success and getattr(result.detected_object, 'x', []))
        if not found:
            self.get_logger().warn("Search found nothing — retrying")
            self._retry_search()
            return

        if self._search_target_type == WskrSearch.Goal.TARGET_BOX:
            self.selected_object = result.detected_object
            self._transition(RobotState.APPROACH_BOX)
        else:
            class_names = [
                str(c) for c in getattr(result.detected_object, 'class_name', [])
            ]
            has_allowed_class = any(c in self._allowed_toy_classes for c in class_names)
            if not has_allowed_class:
                self.get_logger().warn(
                    f"Search found only non-target classes={class_names} -> SEARCH"
                )
                self._retry_search()
                return
            self._transition(RobotState.SELECT)

    def _retry_search(self):
        """Retry current search target.

        If already in the corresponding search state, schedule that state's
        handler again without forcing a same-state transition/log churn.
        """
        if self._search_target_type == WskrSearch.Goal.TARGET_BOX:
            target_state = RobotState.FIND_BOX
            delay = SM_DELAY_FIND_BOX
            handler = self._do_find_box
        else:
            target_state = RobotState.SEARCH
            delay = SM_DELAY_SEARCH
            handler = self._do_search

        if self._get_state() == target_state:
            self._cancel_timer()
            self._timer = self.create_timer(delay, handler)
            return

        self._transition(target_state)

    # ================================================================
    #  SELECT — pick the best object from live YOLO detections
    #
    #  Trigger-style call: the object_selection node picks from its
    #  latest cached YOLO frame. On success -> APPROACH_OBJ,
    #  failure -> SEARCH.
    # ================================================================

    def _do_select(self):
        self._cancel_timer()
        if not self._require_service(self._select_cli, 'select_object_service'):
            return
        self._select_cli.call_async(SelectObject.Request()).add_done_callback(
            self._on_select_result,
        )

    def _on_select_result(self, future):
        """Success -> APPROACH_OBJ, failure -> SEARCH."""
        if self._is_halted():
            return
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Selection failed: {e}")
            self._transition(RobotState.SEARCH)
            return

        if resp.success:
            self.selected_object = resp.selected_obj
            self._transition(RobotState.APPROACH_OBJ)
        else:
            self.get_logger().warn("No object selected -> SEARCH")
            self._transition(RobotState.SEARCH)

    # ================================================================
    #  APPROACH_OBJ / APPROACH_BOX
    #
    #  Both use the ApproachObject action with shared callbacks.
    #  Routing on completion depends on _search_target_type:
    #    proximity + TOY -> GRASP
    #    proximity + BOX -> DROP
    #    no proximity + TOY -> SEARCH  (re-find the target)
    #    no proximity + BOX -> FIND_BOX (keep looking, still holding toy)
    # ================================================================

    def _do_approach_obj(self):
        """Approach the selected toy."""
        self._cancel_timer()
        if not self._require_action(self._approach_ac, 'WSKR/approach_object'):
            return
        goal = ApproachObject.Goal()
        goal.target_type = ApproachObject.Goal.TARGET_TOY
        goal.object_id = int(self._search_target_id)
        goal.selected_obj = self.selected_object
        self._send_approach_goal(goal)

    def _do_approach_box(self):
        """Approach the drop-box ArUco marker."""
        self._cancel_timer()
        if not self._require_action(self._approach_ac, 'WSKR/approach_object'):
            return
        goal = ApproachObject.Goal()
        goal.target_type = ApproachObject.Goal.TARGET_BOX
        goal.object_id = int(self._box_aruco_id)
        if self.selected_object is not None:
            goal.selected_obj = self.selected_object
        self._send_approach_goal(goal)

    def _send_approach_goal(self, goal):
        future = self._approach_ac.send_goal_async(
            goal, feedback_callback=self._on_approach_fb,
        )
        future.add_done_callback(self._on_approach_accepted)

    def _on_approach_fb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(
            f"APPROACH fb: mode={fb.tracking_mode}, "
            f"heading={fb.heading_to_target_deg:.1f}, tracked={fb.visually_tracked}"
        )

    def _on_approach_accepted(self, future):
        goal_handle = future.result()
        self._track(goal_handle)
        if not goal_handle.accepted:
            self.get_logger().warn("Approach goal rejected -> SEARCH")
            self._transition(RobotState.SEARCH)
            return
        goal_handle.get_result_async().add_done_callback(self._on_approach_result)

    def _on_approach_result(self, future):
        if self._is_halted():
            return
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Approach failed: {e}")
            self._transition(RobotState.SEARCH)
            return

        is_box = self._search_target_type == WskrSearch.Goal.TARGET_BOX

        if result.proximity_success:
            self._transition(RobotState.DROP if is_box else RobotState.GRASP)
        elif is_box:
            self.get_logger().warn("Box approach missed proximity -> FIND_BOX")
            self._transition(RobotState.FIND_BOX)
        else:
            self.get_logger().warn("Toy approach missed proximity -> SEARCH")
            self._transition(RobotState.SEARCH)

    # ================================================================
    #  GRASP — pick up the object with the xArm
    #
    #  On success (object in gripper) -> FIND_BOX.
    #  On failure, retry up to _max_grasp_retries then -> SEARCH.
    # ================================================================

    def _do_grasp(self):
        self._cancel_timer()
        if not self._require_action(self._grasp_ac, 'xarm_grasp_action'):
            return

        goal = XArm.Goal()
        goal.id = 0
        goal.selected_obj = self.selected_object

        future = self._grasp_ac.send_goal_async(
            goal, feedback_callback=self._on_grasp_fb,
        )
        future.add_done_callback(self._on_grasp_accepted)

    def _on_grasp_fb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(
            f"GRASP fb: stage={fb.current_stage}, progress={fb.progress:.2f}"
        )

    def _on_grasp_accepted(self, future):
        goal_handle = future.result()
        self._track(goal_handle)
        if not goal_handle.accepted:
            self.get_logger().warn("Grasp goal rejected — retrying")
            self._transition(RobotState.GRASP)
            return
        goal_handle.get_result_async().add_done_callback(self._on_grasp_result)

    def _on_grasp_result(self, future):
        """current_number == 1 means object is in gripper."""
        if self._is_halted():
            return
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Grasp failed: {e}")
            self._grasp_retries = 0
            self._transition(RobotState.SEARCH)
            return

        if result.current_number == 1:
            self._grasp_retries = 0
            self._transition(RobotState.FIND_BOX)
            return

        self._grasp_retries += 1
        if self._grasp_retries <= self._max_grasp_retries:
            self.get_logger().warn(
                f"Grasp retry {self._grasp_retries}/{self._max_grasp_retries}"
            )
            self._transition(RobotState.GRASP)
        else:
            self.get_logger().warn("Grasp retries exhausted -> SEARCH")
            self._grasp_retries = 0
            self._transition(RobotState.SEARCH)

    # ================================================================
    #  DROP — open gripper over the box, then restart collection
    #
    #  Always transitions to SEARCH afterward (continuous collection).
    # ================================================================

    def _do_drop(self):
        self._cancel_timer()
        if not self._require_service(self._gripper_cli, 'open_gripper_service'):
            return
        self._gripper_cli.call_async(Trigger.Request()).add_done_callback(
            self._on_drop_result,
        )

    def _on_drop_result(self, future):
        if self._is_halted():
            return
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().warn(f"Gripper open failed: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Gripper service error: {e}")
        self._transition(RobotState.SEARCH)


# ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
