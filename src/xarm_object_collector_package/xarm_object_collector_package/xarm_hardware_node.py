#!/usr/bin/env python3
"""Single-owner xArm hardware node.

Wraps every XARMController method as a ROS service (or an action for the
one long-running, cancelable op — play_waypoints_dense). No other node
imports controller_class directly; all hardware access goes through here,
so the USB HID device only has one writer.

Services / actions exposed:

    xarm/set_joint_state           robot_interfaces/srv/SetJointState
    xarm/move_joint                robot_interfaces/srv/MoveJoint
    xarm/move_end_effector_count   robot_interfaces/srv/MoveEndEffectorCount
    xarm/get_end_effector_count    robot_interfaces/srv/GetEndEffectorCount
    open_gripper_service           std_srvs/srv/Trigger
    xarm/play_waypoints_dense      robot_interfaces/action/PlayWaypointsDense

All service payloads mirror the XARMController method signatures 1:1.
"""
from __future__ import annotations

import threading
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from robot_interfaces.action import PlayWaypointsDense
from robot_interfaces.srv import (
    GetEndEffectorCount,
    MoveEndEffectorCount,
    MoveJoint,
    SetJointState,
)

from xarm_object_collector_package.controller_class import XARMController

from robot_config.constants import GRIPPER_OPEN_COUNT


class XArmHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__('xarm_hardware_node')

        # Single owner of the USB connection.
        self.xarm = XARMController()
        self.get_logger().info('XARMController initialized; USB device opened.')

        # Serialize hardware access — the xArm USB API isn't thread-safe.
        self._hw_lock = threading.Lock()

        cb = ReentrantCallbackGroup()

        self.create_service(
            SetJointState, 'xarm/set_joint_state', self._on_set_joint_state, callback_group=cb,
        )
        self.create_service(
            MoveJoint, 'xarm/move_joint', self._on_move_joint, callback_group=cb,
        )
        self.create_service(
            MoveEndEffectorCount, 'xarm/move_end_effector_count',
            self._on_move_end_effector_count, callback_group=cb,
        )
        self.create_service(
            GetEndEffectorCount, 'xarm/get_end_effector_count',
            self._on_get_end_effector_count, callback_group=cb,
        )
        self.create_service(
            Trigger, 'open_gripper_service', self._on_open_gripper, callback_group=cb,
        )

        self._play_action = ActionServer(
            self,
            PlayWaypointsDense,
            'xarm/play_waypoints_dense',
            execute_callback=self._on_play_waypoints,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        self.get_logger().info(
            'xarm_hardware_node ready: services (set_joint_state, move_joint, '
            'move_end_effector_count, get_end_effector_count, open_gripper_service) '
            '+ action (play_waypoints_dense).'
        )

    # ------------------------------------------------------------- services
    def _on_set_joint_state(
        self, req: SetJointState.Request, resp: SetJointState.Response,
    ) -> SetJointState.Response:
        duration = list(req.duration_vector) if len(req.duration_vector) > 0 else None
        with self._hw_lock:
            resp.success = bool(self.xarm.set_joint_state(
                list(req.angles),
                servo_ids=list(req.servo_ids),
                duration_vector=duration,
                radians=bool(req.radians),
            ))
        return resp

    def _on_move_joint(
        self, req: MoveJoint.Request, resp: MoveJoint.Response,
    ) -> MoveJoint.Response:
        try:
            with self._hw_lock:
                self.xarm.move_joint(int(req.joint_index), float(req.angle_deg), wait=True)
            resp.success = True
        except Exception as exc:
            self.get_logger().error(f'move_joint failed: {exc}')
            resp.success = False
        return resp

    def _on_move_end_effector_count(
        self, req: MoveEndEffectorCount.Request, resp: MoveEndEffectorCount.Response,
    ) -> MoveEndEffectorCount.Response:
        try:
            with self._hw_lock:
                self.xarm.move_end_effector_count(float(req.count))
            resp.success = True
        except Exception as exc:
            self.get_logger().error(f'move_end_effector_count failed: {exc}')
            resp.success = False
        return resp

    def _on_get_end_effector_count(
        self, _req: GetEndEffectorCount.Request, resp: GetEndEffectorCount.Response,
    ) -> GetEndEffectorCount.Response:
        try:
            with self._hw_lock:
                resp.count = float(self.xarm.get_end_effector_count())
            resp.success = True
        except Exception as exc:
            self.get_logger().error(f'get_end_effector_count failed: {exc}')
            resp.count = 0.0
            resp.success = False
        return resp

    def _on_open_gripper(
        self, _req: Trigger.Request, resp: Trigger.Response,
    ) -> Trigger.Response:
        try:
            with self._hw_lock:
                self.xarm.move_end_effector_count(GRIPPER_OPEN_COUNT)
            resp.success = True
            resp.message = f'Gripper opened (count={GRIPPER_OPEN_COUNT}).'
        except Exception as exc:
            self.get_logger().error(f'open_gripper failed: {exc}')
            resp.success = False
            resp.message = str(exc)
        return resp

    # -------------------------------------------------------------- action
    def _on_play_waypoints(self, goal_handle):
        req = goal_handle.request
        result = PlayWaypointsDense.Result()

        cols = int(req.cols)
        flat = list(req.waypoints_flat)
        if cols <= 0 or len(flat) == 0 or (len(flat) % cols) != 0:
            self.get_logger().error(
                f'play_waypoints_dense invalid shape: len(flat)={len(flat)}, cols={cols}'
            )
            goal_handle.abort()
            result.success = False
            return result

        waypoints = np.array(flat, dtype=float).reshape(-1, cols)

        with self._hw_lock:
            try:
                ok = self.xarm.play_waypoints_dense(
                    waypoints,
                    servo_ids=list(req.servo_ids),
                    cancel_check=lambda: goal_handle.is_cancel_requested,
                )
            except Exception as exc:
                self.get_logger().error(f'play_waypoints_dense raised: {exc}')
                ok = False

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
        elif ok:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
            result.success = False
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XArmHardwareNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
