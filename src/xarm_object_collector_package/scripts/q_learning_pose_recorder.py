#!/usr/bin/env python3
"""
xArm 1s Pose Recorder (pip install xarm)

Records:
- GRASP_POSE:         full 6-servo pose at the grasp position, servo 1 replaced with GRIPPER_OPEN_COUNT
- GRASP_POSE_CLOSED:  same as GRASP_POSE but servo 1 replaced with GRIPPER_CLOSED_COUNT
- MID_CARRY_POSE:     full 6-servo pose at the mid-carry position
- GRIPPER_OPEN_COUNT: servo 1 value when gripper is fully open
- GRIPPER_CLOSED_COUNT: servo 1 value when gripper is fully closed
"""

from __future__ import annotations

import time
from typing import List

import xarm

# ---------- GUI (tkinter) ----------
import tkinter as tk
from tkinter import messagebox


SERVO_IDS = [1, 2, 3, 4, 5, 6]


def ask_ok_cancel(title: str, msg: str) -> bool:
    """Return True if OK, False if Cancel."""
    return messagebox.askokcancel(title, msg)


def ask_yes_no(title: str, msg: str) -> bool:
    """Return True if Yes, False if No."""
    return messagebox.askyesno(title, msg)


def read_all(arm: xarm.Controller) -> List[int]:
    """Read all 6 servo positions as a list in servo-id order."""
    vals = [arm.getPosition(servo_id) for servo_id in SERVO_IDS]
    # Some libs return tuples/lists; ensure plain ints.
    return [int(v) for v in vals]


def main() -> int:
    # Create hidden Tk root so message boxes work cleanly
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)

    # --- Connect ---
    try:
        arm = xarm.Controller("USB")
    except Exception as e:
        messagebox.showerror("Connection Error", f"Failed to connect via USB:\n\n{e}")
        return 1

    # --- Disable all servos (go limp) so the arm can be moved by hand ---
    try:
        arm.servoOff()
    except Exception as e:
        messagebox.showwarning(
            "Servo Off Warning",
            f"Could not disable servos with arm.servoOff().\n"
            f"You can still proceed, but the arm may resist being moved.\n\n{e}",
        )

    time.sleep(0.2)

    # --- Step 1: Grasp position ---
    if not ask_ok_cancel(
        "Pose Recorder – Step 1 of 4",
        "Move the arm to the GRASP position.\n\n"
        "Click OK to record (Cancel to quit)."
    ):
        return 0

    GRASP_POSE = read_all(arm)
    messagebox.showinfo("Recorded", f"Recorded GRASP_POSE:\n{GRASP_POSE}")

    # --- Step 2: Jaws open ---
    if not ask_ok_cancel(
        "Pose Recorder – Step 2 of 4",
        "OPEN the jaws fully.\n\n"
        "Click OK to record GRIPPER_OPEN_COUNT (Cancel to quit)."
    ):
        return 0

    GRIPPER_OPEN_COUNT = int(arm.getPosition(1))
    GRASP_POSE[0] = GRIPPER_OPEN_COUNT  # replace servo 1 in GRASP_POSE with open count
    messagebox.showinfo("Recorded", f"Recorded GRIPPER_OPEN_COUNT = {GRIPPER_OPEN_COUNT}")

    # --- Step 3: Jaws closed ---
    if not ask_ok_cancel(
        "Pose Recorder – Step 3 of 4",
        "CLOSE the jaws fully.\n\n"
        "Click OK to record GRIPPER_CLOSED_COUNT (Cancel to quit)."
    ):
        return 0

    GRIPPER_CLOSED_COUNT = int(arm.getPosition(1))
    messagebox.showinfo("Recorded", f"Recorded GRIPPER_CLOSED_COUNT = {GRIPPER_CLOSED_COUNT}")

    # --- Step 4: Mid-carry position ---
    if not ask_ok_cancel(
        "Pose Recorder – Step 4 of 4",
        "Move the arm to the MID CARRY position.\n\n"
        "Click OK to record (Cancel to quit)."
    ):
        return 0

    MID_CARRY_POSE = read_all(arm)
    messagebox.showinfo("Recorded", f"Recorded MID_CARRY_POSE:\n{MID_CARRY_POSE}")

    GRASP_POSE_CLOSED = list(GRASP_POSE)
    GRASP_POSE_CLOSED[0] = GRIPPER_CLOSED_COUNT

    # --- Print copy/paste-ready output ---
    print("\n\n# ================= COPY/PASTE BELOW =================")
    print(f"GRIPPER_OPEN_COUNT   = {GRIPPER_OPEN_COUNT}")
    print(f"GRIPPER_CLOSED_COUNT = {GRIPPER_CLOSED_COUNT}")
    print(f"GRASP_POSE           = {GRASP_POSE}")
    print(f"GRASP_POSE_CLOSED    = {GRASP_POSE_CLOSED}")
    print(f"MID_CARRY_POSE       = {MID_CARRY_POSE}")
    print("# ================== COPY/PASTE ABOVE =================\n")

    messagebox.showinfo(
        "Done",
        "All poses recorded.\n\n"
        "Copy the values from your terminal output."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())