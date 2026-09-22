#!/usr/bin/env python3

import os
import time
import xarm
import numpy as np

class XARMController:

    ARM_SERVO_IDS = [6, 5, 4, 3, 2]

    def __init__(self):
        """Open USB controller connection and cache initial arm/gripper state."""
        os.environ['LD_LIBRARY_PATH'] = os.getcwd() 
        self.arm = xarm.Controller('USB')  # NEED TO CHANGE THIS 
        self.end_effector = 0.
        self.state = np.array([0.0,0.0,0.0,0.0,0.0])
        self.get_initial_state()
    
    
    def reset(self):
        """Move arm and gripper to a known neutral start pose."""
        self.set_joint_state([0.,-8.,0.,0.,-50.], servo_ids=self.ARM_SERVO_IDS)
        self.move_end_effector(40)
    
    def get_initial_state(self):
        """Read all joint and end-effector positions once at startup."""
        self.state = np.array([self.arm.getPosition(6, degrees=True),
                      self.arm.getPosition(5, degrees=True),
                      self.arm.getPosition(4, degrees=True),
                      self.arm.getPosition(3, degrees=True),
                      self.arm.getPosition(2, degrees=True)
            ])
        self.end_effector = self.arm.getPosition(1, degrees=True)
    
    def update_joints_state(self):
        """Refresh cached 5-joint arm state from hardware encoder readings."""
        self.state = np.array([self.arm.getPosition(6, degrees=True),
                      self.arm.getPosition(5, degrees=True),
                      self.arm.getPosition(4, degrees=True),
                      self.arm.getPosition(3, degrees=True),
                      self.arm.getPosition(2, degrees=True)
        ])
    
    def update_end_effector_state(self):
        """Refresh cached gripper position from hardware."""
        self.end_effector = self.arm.getPosition(1, degrees=True)


    def get_joints_state(self, radians=False):
        """Return latest arm joint vector, optionally converted to radians."""
        self.update_joints_state()
        state = self.state
        if(radians):
            conversion = np.pi/180
            state = conversion * state
        return state
    
    def set_joint_state(self, position_vector, servo_ids, duration_vector=None, radians=False, wait=True):
        """Command a full joint vector to explicit `servo_ids` in one simultaneous multi-servo write.

        Notes:
            The hardware API accepts a single duration for a grouped multi-servo command.
            If `duration_vector` is provided and values differ, the maximum duration is used.
        """
        q = np.asarray(position_vector, dtype=float).reshape(-1)
        if servo_ids is None or len(servo_ids) == 0:
            return False

        servos = [int(servo_id) for servo_id in servo_ids]
        if len(servos) != len(q):
            raise ValueError(
                f"servo_ids has {len(servos)} elements "
                f"but position_vector has {len(q)} elements.\n"
                f"servo_ids={servos}\n"
                f"angles={q.tolist()}"
            )

        if duration_vector is not None:
            durations = np.asarray(duration_vector, dtype=float).reshape(-1)
            if len(durations) != len(q):
                return False
        else:
            durations = None

        conversion = 180/np.pi
        if (radians):
            q *= conversion

        if durations is not None:
            duration_ms = int(round(float(np.max(durations))))
        else:
            duration_ms = 250

        duration_ms = max(1, duration_ms)
        full_cmd = [[servo_id, float(q[servo_index])] for servo_index, servo_id in enumerate(servos)]

        try:
            self.arm.setPosition(full_cmd, duration=duration_ms, wait=wait)
        except Exception:
            return False

        return True

    def move_joint(self, joint_number, angle, duration=250, radians=False, wait=True):
        """Command one joint by index using the internal servo-index mapping."""
        servos = [6,5,4,3,2]
        angle_val = float(angle)
        if(radians):
            conversion = 180/np.pi
            angle_val *= conversion
        self.arm.setPosition(servos[joint_number], angle_val,duration,wait=wait)

    def get_end_effector_state(self, textual=False):
        """Return gripper position numerically or as coarse textual state buckets."""
        self.update_end_effector_state()
        state = self.end_effector
        if(textual):
            if -125 <= state < -90:
                state = 'fully_open'
            elif -90 <= state < -45:
                state = 'partially_open'
            elif -45 <= state < 45:
                state = 'partially_close'
            elif 45 <= state < 90:
                state = 'fully_closed' 
        return state

    def move_end_effector(self, angle):
        """Command gripper angle directly using servo 1."""
        self.arm.setPosition(1,float(angle), wait=True)

    def get_end_effector_count(self):
        """Read gripper count units (non-degree mode) for grasp outcome checks."""
        return float(self.arm.getPosition(1, degrees=False))

    def move_end_effector_count(self, count, duration=250, wait=True):
        """Command gripper in count units with integer-safe duration/count conversion."""
        self.arm.setPosition(1, int(round(count)), int(round(duration)), wait=wait)

    def play_waypoints_dense(self, waypoints, servo_ids, playback_hz=20.0, max_step_deg=1.5, final_settle_ms=500, cancel_check=None):
        """Stream dense-resampled waypoints for explicitly mapped servos.

        Args:
            waypoints: Joint targets with shape (N, M) in degrees. Columns map
                one-to-one to servo_ids.
            servo_ids: Servo IDs corresponding to waypoint columns.
            playback_hz: Streaming frequency in Hz.
            max_step_deg: Maximum allowed per-joint delta between streamed commands.
            final_settle_ms: Blocking settle duration for final command.
            cancel_check: Optional callable returning True to cancel playback.

        Returns:
            bool: True on successful completion, False on cancel or command failure.
        """
        trajectory = np.asarray(waypoints, dtype=float)

        if trajectory.ndim != 2:
            return False

        if servo_ids is None or len(servo_ids) == 0:
            return False

        servos = [int(servo_id) for servo_id in servo_ids]
        if trajectory.shape[1] != len(servos):
            return False

        if trajectory.shape[0] == 0:
            return True
        if not np.isfinite(trajectory).all():
            return False

        playback_hz = float(playback_hz)
        max_step_deg = float(max_step_deg)
        final_settle_ms = int(final_settle_ms)
        if playback_hz <= 0.0 or max_step_deg <= 0.0:
            return False

        max_step_deg = max(0.1, max_step_deg)
        final_settle_ms = max(1, final_settle_ms)

        current = np.asarray(
            [self.arm.getPosition(servo_id, degrees=True) for servo_id in servos],
            dtype=float,
        ).reshape(len(servos))

        dense_points = []
        previous = current.copy()

        for target in trajectory:
            target = np.asarray(target, dtype=float).reshape(len(servos))
            delta = target - previous
            max_delta = float(np.max(np.abs(delta)))
            steps = max(1, int(np.ceil(max_delta / max_step_deg)))

            for alpha in np.linspace(0.0, 1.0, steps + 1, endpoint=True)[1:]:
                dense_points.append(previous + (delta * alpha))

            previous = target

        if len(dense_points) == 0:
            return True

        interval = 1.0 / playback_hz
        step_duration_ms = max(1, int(round(interval * 1000.0)))
        stream_start = time.time()

        for index, target in enumerate(dense_points):
            if callable(cancel_check) and bool(cancel_check()):
                return False

            target_time = stream_start + (index * interval)
            sleep_time = target_time - time.time()
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            full_cmd = [[servo_id, float(target[servo_index])] for servo_index, servo_id in enumerate(servos)]
            self.arm.setPosition(full_cmd, duration=step_duration_ms, wait=False)

        final_target = np.asarray(dense_points[-1], dtype=float).reshape(len(servos))
        settle_duration = max(final_settle_ms, 2 * step_duration_ms)
        final_cmd = [[servo_id, float(final_target[servo_index])] for servo_index, servo_id in enumerate(servos)]
        self.arm.setPosition(final_cmd, duration=settle_duration, wait=True)

        return True
    