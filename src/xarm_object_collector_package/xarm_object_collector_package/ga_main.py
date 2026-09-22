#!/usr/bin/env python3
"""
Merged GA main script.

Base structure/config taken from GA_main_NoArgs.py (the primary script).
Simulation/RViz plumbing (viz_publisher wiring, publish_generation_trajectory,
interpolate_trajectory_for_viz, confirm_robot_motion_start) is unchanged from
both originals since they were identical.

Hardware execution now goes through XARMController's public API
(set_joint_state / move_end_effector / play_waypoints_dense) instead of
reaching into the raw xarm.Controller via `arm.arm`. This also fixes two bugs
present in both source scripts:
  - `arm.arm.move_end_effector(...)` doesn't exist (move_end_effector lives on
    XARMController, not on the raw xarm.Controller) -> would raise AttributeError.
  - ga_main.py called `arm.play_waypoints_dense(...)` without the required
    `servo_ids` argument -> would raise TypeError.
The old manual resample_trajectory_for_hardware/build_full_joint_command
helpers from GA_main_NoArgs.py were dropped since that resampling logic
already lives in controller_class.play_waypoints_dense.
"""

import importlib
import time
import numpy as np
from genetic_algorithm import GeneAlgo

# ----------------------------
# IDE Runtime Configuration
# ----------------------------
GOAL_X = 150  # Target X position in millimeters
GOAL_Y = 0  # Target Y position in millimeters
GOAL_Z = 0  # Target Z position in millimeters

VIZ = True  # Enable ROS2 visualization
EXECUTE_HARDWARE = True  # Execute trajectory on physical robot
OPERATE_HAND = True  # Control gripper during execution (NOT USEFUL UNTIL Q-TABLE IMPLEMENTED)
NO_HOME = False  # Skip moving to home/mid-carry position

## Visualization and execution parameters (do not change unless you know what you're doing)
PLAYBACK_HZ = 30.0  # Visualization playback frequency in Hz
INTERP_STEPS = 5  # Interpolation steps between GA waypoints
VIZ_SKIP_GENS = 9  # Visualization cadence (0=every gen, N=every N+1th)
TOL = 2.0  # Joint tolerance for reaching target in degrees
TIMEOUT_PER_STEP = 15.0  # Timeout per step during hardware execution in seconds
HARDWARE_PLAYBACK_HZ = 20.0  # Hardware trajectory streaming frequency in Hz
HARDWARE_MAX_STEP_DEG = 1.5  # Maximum per-joint delta per hardware command


# ----------------------------
# GA
# ----------------------------
def infer_yaw_deg(goal_x, goal_y):
    """Infer the fixed base yaw from the target azimuth in the X-Y plane."""
    return float(np.degrees(np.arctan2(float(goal_y), float(goal_x))))


def build_mid_carry(yaw_deg):
    """Build the default home/carry position including the fixed yaw servo."""
    return [
        [6, float(yaw_deg)],
        [5, -14.25],
        [4, 76.75],
        [3, 0.0],
    ]


def run_ga(goal_x, goal_y, goal_z, yaw_deg=None):
    if yaw_deg is None:
        yaw_deg = infer_yaw_deg(goal_x, goal_y)
    solver = GeneAlgo(yaw_deg=yaw_deg)
    goal = np.array([float(goal_x), float(goal_y), float(goal_z)], dtype=float)
    return solver.solve(goal, yaw_deg=yaw_deg)


# ----------------------------
# Helpers
# ----------------------------
def build_controller_waypoints(current_full, motor_sets, yaw_deg=0.0):
    """Build (N, 5) waypoints for XARMController.play_waypoints_dense.

    The controller expects joints ordered [arm6, arm5, arm4, arm3, arm2] in
    degrees. GA output may be [arm5, arm4, arm3] or [arm6, arm5, arm4, arm3].
    arm2 is held at the robot's current value.
    """
    current_full = np.asarray(current_full, dtype=float).reshape(5)
    arm2_hold = float(current_full[4])
    trajectory = np.asarray(motor_sets, dtype=float)

    if trajectory.ndim != 2 or trajectory.shape[0] == 0:
        return np.zeros((0, 5), dtype=float)

    if trajectory.shape[1] == 3:
        arm6 = np.full((trajectory.shape[0], 1), float(yaw_deg), dtype=float)
        arm2 = np.full((trajectory.shape[0], 1), arm2_hold, dtype=float)
        return np.hstack((arm6, trajectory, arm2))

    if trajectory.shape[1] >= 4:
        arm6 = trajectory[:, 0:1]
        triplet = trajectory[:, 1:4]
        arm2 = np.full((trajectory.shape[0], 1), arm2_hold, dtype=float)
        return np.hstack((arm6, triplet, arm2))

    return np.zeros((0, 5), dtype=float)


def wait_until_close(controller, target_full, tol=2.0, timeout=15.0):
    """Optional verification for reaching target (can be skipped for smooth motion)."""
    target = target_full[0:4]
    start = time.time()
    last_error = float("inf")

    while time.time() - start < timeout:
        current = np.array(controller.get_joints_state(), dtype=float)
        error = np.linalg.norm(current[0:4] - target)
        print(f"Current: {current[0:4].tolist()} | Target: {target.tolist()} | Error: {error:.3f}")

        if error < tol:
            return True

        # detect stall
        if abs(error - last_error) < 0.01:
            print("Warning: no progress detected")
            return False

        last_error = error
        time.sleep(0.1)

    return False


def interpolate_steps(start, end, num_points=10):
    """Generate a list of intermediate joint positions from start to end."""
    return [start + (end - start) * (i / num_points) for i in range(1, num_points + 1)]


def interpolate_trajectory_for_viz(motor_sets_deg, interp_steps=5):
    """Convert GA waypoints in degrees into an interpolated radians trajectory."""
    trajectory_deg = np.asarray(motor_sets_deg, dtype=float)
    if trajectory_deg.size == 0:
        return np.zeros((0, 3), dtype=float)

    # Extract only the joint triplet (skip yaw at column 0)
    if trajectory_deg.ndim == 2 and trajectory_deg.shape[1] == 4:
        trajectory_deg = trajectory_deg[:, 1:4]

    waypoints = np.vstack((GeneAlgo.INITIAL_POS, trajectory_deg))
    frames = []

    for index in range(len(waypoints) - 1):
        segment = np.linspace(waypoints[index], waypoints[index + 1], int(interp_steps) + 1, endpoint=True)
        if index > 0:
            segment = segment[1:]
        frames.append(segment)

    if not frames:
        return np.zeros((0, 3), dtype=float)

    return np.deg2rad(np.vstack(frames))


def publish_generation_trajectory(viz_publisher, motor_sets_deg, interp_steps=5, yaw_deg=0.0, arm2_rad=0.0, arm1_rad=0.0):
    """Interpolate the raw GA motor sets and publish them through the viz node."""
    trajectory_rad = interpolate_trajectory_for_viz(motor_sets_deg, interp_steps=interp_steps)
    if trajectory_rad.size > 0:
        viz_publisher.publish_trajectory(
            trajectory_rad,
            yaw_rad=np.deg2rad(yaw_deg),
            arm2_rad=arm2_rad,
            arm1_rad=arm1_rad,
        )


def confirm_robot_motion_start():
    """Block on a GUI dialog and return True only when the user confirms movement."""
    try:
        import tkinter as tk
        from tkinter import messagebox

        root = tk.Tk()
        root.withdraw()
        confirmed = messagebox.askokcancel(
            "Confirm Robot Motion",
            "The robot is about to start moving.\n\nClick OK to proceed.",
        )
        root.destroy()
        return bool(confirmed)
    except Exception as exc:
        print(f"Unable to show GUI confirmation dialog: {exc}")
        print("Aborting hardware execution for safety.")
        return False


# ----------------------------
# Execution with smooth motion
# ----------------------------
def execute_on_robot(motor_sets, yaw_deg=0.0, tol=2.0, timeout_per_step=15.0, move_home_first=True):
    from controller_class import XARMController

    arm = XARMController()
    print("Controller connected")
    mid_carry = build_mid_carry(yaw_deg)

    # Move to known start via the controller's public API (not the raw xarm handle).
    if move_home_first:
        print("Moving to MID_CARRY:", mid_carry)
        mid_carry_ids = [entry[0] for entry in mid_carry]
        mid_carry_angles = [entry[1] for entry in mid_carry]
        arm.set_joint_state(
            mid_carry_angles,
            servo_ids=mid_carry_ids,
            duration_vector=[1000] * len(mid_carry_ids),
            wait=True,
        )

    if OPERATE_HAND:
        print("Opening gripper")
        arm.move_end_effector(-70)

    # Get real starting state
    current_full = np.array(arm.get_joints_state(), dtype=float)
    current_yaw_and_triplet = current_full[0:4].copy()
    print("Starting joints [arm6, arm5, arm4, arm3]:", current_yaw_and_triplet.tolist())

    waypoints_5d = build_controller_waypoints(current_full, motor_sets, yaw_deg=yaw_deg)
    playback_hz = max(1.0, float(HARDWARE_PLAYBACK_HZ))
    print(
        f"Streaming trajectory at {playback_hz:.1f} Hz using controller dense playback. "
        f"Raw waypoints: {len(motor_sets)}"
    )

    playback_ok = arm.play_waypoints_dense(
        waypoints_5d,
        servo_ids=XARMController.ARM_SERVO_IDS,
        playback_hz=playback_hz,
        max_step_deg=HARDWARE_MAX_STEP_DEG,
        final_settle_ms=250,
    )
    if not playback_ok:
        raise RuntimeError("Hardware playback failed or was canceled")

    if len(motor_sets) > 0:
        final_waypoint = np.asarray(waypoints_5d[-1], dtype=float)
        final_cmd = [[servo_id, float(final_waypoint[i])] for i, servo_id in enumerate(XARMController.ARM_SERVO_IDS)]
        print("Commanded final joint angles (deg):", final_cmd)

        actual_final_state = np.asarray(arm.get_joints_state(), dtype=float)
        actual_servo_angles = [
            [servo_id, float(actual_final_state[i])] for i, servo_id in enumerate(XARMController.ARM_SERVO_IDS)
        ]
        print("Actual final servo angles (deg):", actual_servo_angles)

    if OPERATE_HAND:
        print("Closing gripper")
        arm.move_end_effector(30)

    print("\nExecution complete")


# ----------------------------
# MAIN
# ----------------------------
def main():
    """Run GA and optionally stream generation-end trajectories to /joint_states."""
    yaw_deg = infer_yaw_deg(GOAL_X, GOAL_Y)
    print(f"Running GA for goal=({GOAL_X}, {GOAL_Y}, {GOAL_Z}), yaw={yaw_deg} deg")

    viz_publisher = None
    rclpy_module = None
    viz_callback = None
    viz_gripper_arm1_rad = 0.0
    viz_gripper_arm2_rad = 0.0

    if VIZ:
        try:
            rclpy_module = importlib.import_module("rclpy")
            from xarm_object_collector_package.viz_publisher import VizPublisher
        except ImportError as exc:
            raise RuntimeError("VIZ=True requires ROS2 Python packages (rclpy, sensor_msgs, visualization_msgs).") from exc

        # Create a lightweight ROS2 node dedicated to trajectory playback.
        rclpy_module.init()
        viz_publisher = VizPublisher(playback_hz=PLAYBACK_HZ)
        viz_publisher.publish_goal_marker(GOAL_X, GOAL_Y, GOAL_Z)

        if OPERATE_HAND and not EXECUTE_HARDWARE:
            viz_gripper_arm1_rad = float(VizPublisher.GRIPPER_OPEN_RAD)
            viz_publisher.publish_gripper_state(arm1_rad=viz_gripper_arm1_rad, arm2_rad=viz_gripper_arm2_rad)

        viz_callback = lambda motor_sets_deg: publish_generation_trajectory(
            viz_publisher,
            motor_sets_deg,
            interp_steps=INTERP_STEPS,
            yaw_deg=yaw_deg,
            arm2_rad=viz_gripper_arm2_rad,
            arm1_rad=viz_gripper_arm1_rad,
        )

    try:
        solver = GeneAlgo(
            viz_enabled=VIZ,
            viz_callback=viz_callback,
            viz_skip_gens=VIZ_SKIP_GENS,
            yaw_deg=yaw_deg,
        )
        goal = np.array([float(GOAL_X), float(GOAL_Y), float(GOAL_Z)], dtype=float)
        motor_sets = solver.solve(goal, yaw_deg=yaw_deg)

        if motor_sets.size == 0:
            raise RuntimeError("GA returned empty result")

        print("Trajectory length:", len(motor_sets))
        print("First:", motor_sets[0].tolist())
        print("Last:", motor_sets[-1].tolist())

        if not EXECUTE_HARDWARE:
            if VIZ and OPERATE_HAND and viz_publisher is not None:
                viz_publisher.publish_gripper_state(arm1_rad=float(VizPublisher.GRIPPER_CLOSED_RAD), arm2_rad=viz_gripper_arm2_rad)
            print("GA only mode complete")
            return

        if not confirm_robot_motion_start():
            print("Motion canceled by user.")
            return

        execute_on_robot(
            motor_sets,
            yaw_deg=yaw_deg,
            tol=TOL,
            timeout_per_step=TIMEOUT_PER_STEP,
            move_home_first=(not NO_HOME),
        )
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting cleanly.")
    finally:
        if viz_publisher is not None:
            try:
                viz_publisher.destroy_node()
            except Exception:
                pass
        if rclpy_module is not None:
            try:
                # Ctrl+C or ROS signal handlers may have already shut the context down.
                if hasattr(rclpy_module, "ok"):
                    if rclpy_module.ok():
                        rclpy_module.shutdown()
                else:
                    rclpy_module.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()