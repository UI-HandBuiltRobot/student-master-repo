"""Train a simple Q table for robot-arm grasping (optimized).

Purpose:
- Let the robot repeatedly attempt grasps, reward success, penalize failure,
  and save the learned Q values after every trial.

Inputs:
- Camera images from the USB camera selected by constants.CAMERA_INDEX.
- Roboflow detections from inference_sdk using the shared model constants.
- User confirmation dialogs shown with tkinter.

Outputs:
- A CSV Q table saved to constants.Q_TABLE_FILE.

Dependencies:
- xarm
- inference_sdk
- cv2
- numpy
- pandas
- tkinter
- constants.py
"""

import os
import re
import tempfile
import tkinter as tk

import cv2
import numpy as np
import pandas as pd
import xarm
from inference_sdk import InferenceHTTPClient

from robot_config.constants import (
    ASPECT_BINS,
    ENABLE_IMG_DEG_ROTATION_SIGN_FEATURE,
    ENABLE_GUI_UPDATES,
    CAMERA_DEVICE,
    EXPLORATION_WEIGHT,
    GRASP_POSE,
    GRASP_POSE_CLOSED,
    WRIST_SERVO_ID,
    ARM_SERVO_IDS,
    GRIPPER_BLOCKED_THRESHOLD,
    GRIPPER_CLOSED_COUNT,
    GRIPPER_OPEN_COUNT,
    GRIPPER_SERVO_INDEX,
    LEARNING_RATE,
    MID_CARRY_POSE,
    Q_TABLE_FILE,
    ROBOFLOW_API_KEY,
    ROBOFLOW_MODEL_ID,
    ROBOFLOW_VERSION,
    TRAIN_PER_CLASS,
    TRIALS_PER_BIN,
    WRIST_ANGLES,
    IMG_ROTATION_ANGLE,
)


# Resolve Q_TABLE_FILE to an absolute path anchored on this script's location,
# not on the CWD or on constants.py's location. This places the Q table in
# <xarm_object_collector_package>/data/ regardless of where the script is run
# from (src/, build/, or install/). The Q_TABLE_FILE env var is honored as-is
# so the user can override the location.
if not os.path.isabs(Q_TABLE_FILE):
    Q_TABLE_FILE = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),  # .../xarm_object_collector_package/scripts
        Q_TABLE_FILE,                                # default is ../data/q_table.csv
    )


def build_state_bins():
    """Return the state bins, optionally mirrored to include signed bins."""
    base_bins = [abs(float(aspect_bin)) for aspect_bin in ASPECT_BINS]
    base_bins = list(dict.fromkeys(base_bins))  # preserve order, remove duplicates

    if not ENABLE_IMG_DEG_ROTATION_SIGN_FEATURE:
        return base_bins

    mirrored_bins = list(base_bins)
    mirrored_bins.extend([-aspect_bin for aspect_bin in base_bins if aspect_bin != 0.0])
    return mirrored_bins


def show_ok_dialog(message_text, window_title="Q-Learning Training"):
    """Show a simple dialog and return True only if the user clicked OK."""
    dialog_result = {"ok_clicked": False}

    root_window = tk.Tk()
    root_window.title(window_title)
    root_window.geometry("660x380")
    root_window.resizable(False, False)

    message_label = tk.Label(root_window, text=message_text, wraplength=610, justify="left")
    message_label.pack(padx=20, pady=20)

    def on_ok_clicked():
        dialog_result["ok_clicked"] = True
        root_window.destroy()

    ok_button = tk.Button(root_window, text="OK", width=14, command=on_ok_clicked)
    ok_button.pack(pady=10)
    root_window.bind("<Return>", lambda event: on_ok_clicked())
    ok_button.focus_set()

    root_window.protocol("WM_DELETE_WINDOW", root_window.destroy)
    root_window.mainloop()
    return dialog_result["ok_clicked"]


def confirm_or_continue(message_text, window_title="Q-Learning Training"):
    """Show dialog only when GUI updates are enabled; otherwise auto-continue."""
    if not ENABLE_GUI_UPDATES:
        print(f"[GUI disabled] {message_text}")
        return True
    return show_ok_dialog(message_text, window_title=window_title)


def connect_to_arm():
    """Connect to the xarm over USB."""
    print("Connecting to robot arm on USB...")
    arm_controller = xarm.Controller("USB")
    print("Connected to robot arm.")
    return arm_controller


def build_inference_model_id():
    """Build the model ID string expected by inference_sdk."""
    return f"{ROBOFLOW_MODEL_ID}/{ROBOFLOW_VERSION}"


def connect_to_inference_client():
    """Create an inference_sdk HTTP client for Roboflow serverless inference."""
    print("Connecting to Roboflow serverless inference API...")
    inference_client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",
        api_key=ROBOFLOW_API_KEY,
    )
    print("Connected to inference client.")
    return inference_client


def open_camera():
    # THIS HAS BEEN EDITED MUST CHECK
    print(f"Opening USB camera at device {CAMERA_DEVICE} (V4L2)...")
    camera_capture = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

    if not camera_capture.isOpened():
        raise RuntimeError(f"Could not open camera at {CAMERA_DEVICE}.")
    print("Camera opened.")

    return camera_capture


def load_or_initialize_q_table(q_table_file=Q_TABLE_FILE):
    """Load an existing Q table or create/reindex one for current settings."""
    expected_index = pd.Index(build_state_bins(), dtype=float)
    expected_columns = pd.Index([float(a) for a in WRIST_ANGLES], dtype=float)

    def create_new_q_table():
        print(f"Creating a new Q table at {q_table_file}...")
        values = np.random.uniform(0.0, 0.01, size=(len(expected_index), len(expected_columns)))
        q = pd.DataFrame(values, index=expected_index, columns=expected_columns)
        q.to_csv(q_table_file)
        print("Created new Q table.")
        return q

    if not os.path.exists(q_table_file):
        return create_new_q_table()

    if os.path.getsize(q_table_file) == 0:
        print(f"Q table file {q_table_file} is empty. Reinitializing.")
        return create_new_q_table()

    print(f"Loading existing Q table from {q_table_file}...")
    try:
        q_table = pd.read_csv(q_table_file, index_col=0)
    except pd.errors.EmptyDataError:
        print(f"Q table file {q_table_file} has no CSV data. Reinitializing.")
        return create_new_q_table()

    if q_table.empty:
        print(f"Q table file {q_table_file} has no rows. Reinitializing.")
        return create_new_q_table()

    q_table.index = q_table.index.astype(float)
    q_table.columns = q_table.columns.astype(float)
    if q_table.index.equals(expected_index) and q_table.columns.equals(expected_columns):
        print("Loaded Q table.")
        return q_table

    print("Existing Q table schema does not match current constants. Reindexing table...")
    q_table = q_table.reindex(index=expected_index, columns=expected_columns)
    missing_mask = q_table.isna()
    if missing_mask.any().any():
        fill = pd.DataFrame(np.random.uniform(0.0, 0.01, size=q_table.shape), index=q_table.index, columns=q_table.columns)
        q_table = q_table.where(~missing_mask, fill)
    q_table.to_csv(q_table_file)
    print("Saved reindexed Q table to match current constants.")
    print("Loaded Q table.")
    return q_table


def extract_object_class_name(detection):
    """Return the detected object-class label, defaulting to unknown."""
    return str(
        detection.get("class")
        or detection.get("class_name")
        or detection.get("label")
        or "unknown"
    ).strip() or "unknown"


def sanitize_object_class_name(object_class):
    """Convert a detected class label into a filesystem-safe table suffix."""
    sanitized_name = re.sub(r"[^0-9a-zA-Z]+", "_", str(object_class).strip().lower())
    return sanitized_name.strip("_") or "unknown"


def build_q_table_file_path(object_class=None):
    """Return the Q-table file path for the detected object class."""
    if not TRAIN_PER_CLASS or object_class is None:
        return Q_TABLE_FILE

    file_root, file_extension = os.path.splitext(Q_TABLE_FILE)
    return f"{file_root}_{sanitize_object_class_name(object_class)}{file_extension or '.csv'}"


def get_q_table_bundle(object_class, q_table_cache, visited_table_cache, observed_bin_counts_cache):
    """Return the cached Q table, visited map, and counters for one object class."""
    q_table_file = build_q_table_file_path(object_class)
    if q_table_file not in q_table_cache:
        q_table_cache[q_table_file] = load_or_initialize_q_table(q_table_file)
        visited_table_cache[q_table_file] = create_visited_table(q_table_cache[q_table_file])
        observed_bin_counts_cache[q_table_file] = {
            float(aspect_bin): 0 for aspect_bin in q_table_cache[q_table_file].index.tolist()
        }
        if TRAIN_PER_CLASS:
            print(f"Using per-class Q table for '{object_class}': {q_table_file}")

    return (
        q_table_file,
        q_table_cache[q_table_file],
        visited_table_cache[q_table_file],
        observed_bin_counts_cache[q_table_file],
    )


def create_visited_table(q_table):
    """Create a volatile visited table with the same shape as the Q table."""
    # This table is intentionally volatile and resets every training run. It is
    # only used to ensure every action is attempted at least once per state
    # before exploitation begins to dominate action selection.
    return pd.DataFrame(False, index=q_table.index, columns=q_table.columns)


def save_frame_to_temporary_image(frame_bgr):
    """Write one camera frame to a temporary image file for inference upload."""
    temporary_file_handle = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
    temporary_file_handle.close()
    cv2.imwrite(temporary_file_handle.name, frame_bgr)
    return temporary_file_handle.name


def run_inference(inference_client, image_path):
    """Run inference_sdk inference on one saved image file."""
    return inference_client.infer(image_path, model_id=build_inference_model_id())


def extract_highest_confidence_detection(prediction_json):
    """Return the highest-confidence bounding box from the Roboflow response, or None if no predictions."""
    predictions = prediction_json.get("predictions", [])
    if not predictions:
        return None
    return max(predictions, key=lambda prediction: prediction.get("confidence", 0.0))


def show_image_with_bounding_box(
    frame_bgr,
    detection,
    window_name="Object Detection Preview",
    aspect_ratio_label_override=None,
):
    """Display one preview image with optional detection box and AR label."""
    frame_with_box = frame_bgr.copy()

    if detection is not None:
        x, y = float(detection.get("x", 0)), float(detection.get("y", 0))
        width, height = float(detection.get("width", 0)), float(detection.get("height", 0))
        confidence = float(detection.get("confidence", 0.0))
        left, top = int(x - width / 2), int(y - height / 2)
        right, bottom = int(x + width / 2), int(y + height / 2)
        cv2.rectangle(frame_with_box, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame_with_box, f"Confidence: {confidence:.2f}", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        observed_ar = width / height if height > 0 else 0.0
        aspect_ratio_label = aspect_ratio_label_override or f"Observed aspect ratio: {observed_ar:.4f}"
    else:
        cv2.putText(frame_with_box, "No object detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        aspect_ratio_label = aspect_ratio_label_override or "Observed aspect ratio: N/A"

    cv2.putText(
        frame_with_box,
        aspect_ratio_label,
        (20, frame_with_box.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 0),
        2,
    )
    cv2.imshow(window_name, frame_with_box)
    for _ in range(10):
        cv2.waitKey(30)


def flush_camera_buffer(camera_capture, num_frames=5):
    """Discard buffered frames so the next read() returns a fresh live image."""
    for _ in range(num_frames):
        camera_capture.grab()


def infer_highest_confidence_detection_from_frame(frame_bgr, inference_client):
    """Run inference on a frame and return the highest-confidence detection or None."""
    temporary_image_path = save_frame_to_temporary_image(frame_bgr)
    try:
        prediction_json = run_inference(inference_client, temporary_image_path)
    finally:
        if os.path.exists(temporary_image_path):
            os.remove(temporary_image_path)
    return extract_highest_confidence_detection(prediction_json)


def capture_and_infer(camera_capture, inference_client):
    """Capture one frame, infer one detection, and convert it into a state bin."""
    flush_camera_buffer(camera_capture)
    frame_was_captured, frame_bgr = camera_capture.read()
    if not frame_was_captured:
        raise RuntimeError("Failed to capture a frame from the USB camera.")

    best_detection = infer_highest_confidence_detection_from_frame(frame_bgr, inference_client)
    
    # Show preview with bounding box (or without if no detection)
    show_image_with_bounding_box(frame_bgr, best_detection)
    
    # Raise error if no detection was found
    if best_detection is None:
        raise RuntimeError("No object was detected in the camera image.")

    object_class = extract_object_class_name(best_detection)
    
    bounding_box_width = float(best_detection["width"])
    bounding_box_height = float(best_detection["height"])
    if bounding_box_height <= 0.0:
        raise RuntimeError("The detected bounding box height was zero or negative.")

    raw_aspect_ratio = bounding_box_width / bounding_box_height

    # The aspect ratio is used as the state variable because it is easy for
    # students to understand and easy to compute from a detector bounding box.
    positive_bins = [abs(float(aspect_bin)) for aspect_bin in ASPECT_BINS]
    nearest_aspect_bin = min(positive_bins, key=lambda candidate_bin: abs(candidate_bin - raw_aspect_ratio))

    if ENABLE_IMG_DEG_ROTATION_SIGN_FEATURE:
        # Optional N-degree rotation feature.
        # Compare rotated-vs-original AR directly to assign sign.
        rotation_center = (frame_bgr.shape[1] / 2.0, frame_bgr.shape[0] / 2.0)
        rotation_matrix = cv2.getRotationMatrix2D(rotation_center, IMG_ROTATION_ANGLE, 1.0)
        rotated_frame = cv2.warpAffine(
            frame_bgr,
            rotation_matrix,
            (frame_bgr.shape[1], frame_bgr.shape[0]),
        )
        rotated_detection = infer_highest_confidence_detection_from_frame(rotated_frame, inference_client)
        signed_aspect_ratio = raw_aspect_ratio
        if rotated_detection is None:
            # Show second preview immediately after rotation inference.
            show_image_with_bounding_box(
                rotated_frame,
                rotated_detection,
                window_name=f"Object Detection Preview ({IMG_ROTATION_ANGLE} deg)",
                aspect_ratio_label_override="Signed aspect ratio: N/A",
            )
            raise RuntimeError(f"No object was detected in the {IMG_ROTATION_ANGLE}-degree rotated image.")

        rotated_width = float(rotated_detection["width"])
        rotated_height = float(rotated_detection["height"])
        if rotated_height <= 0.0:
            raise RuntimeError("The rotated detected bounding box height was zero or negative.")

        rotated_aspect_ratio = rotated_width / rotated_height
        rotated_minus_original = rotated_aspect_ratio - raw_aspect_ratio
        if rotated_minus_original >= 0.0:
            nearest_aspect_bin = abs(nearest_aspect_bin)
        else:
            nearest_aspect_bin = -abs(nearest_aspect_bin)
        signed_aspect_ratio = raw_aspect_ratio if nearest_aspect_bin >= 0.0 else -raw_aspect_ratio

        # Show second preview immediately after rotation inference.
        show_image_with_bounding_box(
            rotated_frame,
            rotated_detection,
            window_name=f"Object Detection Preview ({IMG_ROTATION_ANGLE} deg)",
            aspect_ratio_label_override=f"Signed aspect ratio: {signed_aspect_ratio:.4f}",
        )


    return nearest_aspect_bin, raw_aspect_ratio, object_class


POSE_EXCLUDED_SERVO_IDS = {1, 2}  # Gripper and wrist are set independently; exclude from pose moves.


def pose_to_servo_commands(pose_positions):
    """Convert a saved pose list into the servo/value structure expected by xarm.

    Servo IDs in POSE_EXCLUDED_SERVO_IDS (gripper and wrist) are omitted so
    that independently commanded wrist angles and jaw positions are not
    overwritten when the arm moves to a stored pose.
    """
    return [
        [servo_id, pose_positions[index]]
        for index, servo_id in enumerate(ARM_SERVO_IDS)
        if servo_id not in POSE_EXCLUDED_SERVO_IDS
    ]


def move_to_pose(arm_controller, pose_positions, pose_name):
    """Move the arm to one of the recorded poses."""
    print(f"Moving to {pose_name} pose...")
    arm_controller.setPosition(pose_to_servo_commands(pose_positions), wait=True)


def set_wrist_angle(arm_controller, wrist_angle):
    """Set the wrist servo to the selected candidate angle."""
    print(f"Setting wrist servo to {wrist_angle} degrees...")
    arm_controller.setPosition(WRIST_SERVO_ID, float(wrist_angle), wait=True)


def close_gripper(arm_controller):
    """Command the jaws to the configured fully closed position."""
    gripper_servo_id = GRIPPER_SERVO_INDEX + 1
    print("Closing gripper...")
    arm_controller.setPosition(gripper_servo_id, GRIPPER_CLOSED_COUNT, wait=True)


def open_gripper(arm_controller):
    """Command the jaws to the configured fully open position."""
    gripper_servo_id = GRIPPER_SERVO_INDEX + 1
    print("Opening gripper...")
    arm_controller.setPosition(gripper_servo_id, GRIPPER_OPEN_COUNT, wait=True)


def read_gripper_position(arm_controller):
    """Read the current gripper servo position in controller units."""
    gripper_servo_id = GRIPPER_SERVO_INDEX + 1
    return float(arm_controller.getPosition(gripper_servo_id))


def main():
    print("Starting Q-learning training.")

    arm_controller = connect_to_arm()
    inference_client = connect_to_inference_client()
    camera_capture = open_camera()
    q_table_cache = {}
    visited_table_cache = {}
    observed_bin_counts_cache = {}
    total_trials = len(build_state_bins()) * TRIALS_PER_BIN

    if not TRAIN_PER_CLASS:
        get_q_table_bundle(None, q_table_cache, visited_table_cache, observed_bin_counts_cache)

    try:
        for trial_number in range(1, total_trials + 1):
            prompt_message = (
                f"Trial {trial_number} of {total_trials}. "
                "Place the object in any orientation, then click OK when ready."
            )
            if not confirm_or_continue(prompt_message):
                print("Training stopped by user.")
                raise SystemExit(0)

            while True:
                cv2.destroyAllWindows()
                print(f"Capturing image for trial {trial_number} of {total_trials}...")
                try:
                    observed_aspect_bin, raw_aspect_ratio, object_class = capture_and_infer(
                        camera_capture,
                        inference_client,
                    )
                    break
                except RuntimeError as detection_error:
                    print(f"Detection failed: {detection_error}")
                    if not confirm_or_continue(
                        f"No object detected. Please reposition the object and try again.\n\n({detection_error})"
                    ):
                        print("Training stopped by user.")
                        raise SystemExit(0)

            q_table_file, q_table, visited_table, observed_bin_counts = get_q_table_bundle(
                object_class if TRAIN_PER_CLASS else None,
                q_table_cache,
                visited_table_cache,
                observed_bin_counts_cache,
            )

            observed_bin_counts[float(observed_aspect_bin)] += 1
            print(
                f"Observed raw aspect ratio {raw_aspect_ratio:.4f}, mapped to nearest bin {observed_aspect_bin}."
            )
            if TRAIN_PER_CLASS:
                print(f"Detected object class '{object_class}', using Q table {q_table_file}.")
            print(f"Observed-bin count for {observed_aspect_bin}: {observed_bin_counts[float(observed_aspect_bin)]}")

            # This exploration-adjusted score subtracts a penalty from any
            # action that was already tried for this state so the learner is
            # pushed to cover all actions before it settles into exploitation.
            exploration_adjusted_scores = {}
            for wrist_angle in WRIST_ANGLES:
                current_q_value = float(q_table.loc[observed_aspect_bin, wrist_angle])
                visited_penalty = EXPLORATION_WEIGHT * (
                    1 if bool(visited_table.loc[observed_aspect_bin, wrist_angle]) else 0
                )
                exploration_adjusted_scores[wrist_angle] = current_q_value - visited_penalty

            selected_wrist_angle = max(
                exploration_adjusted_scores,
                key=lambda candidate_angle: exploration_adjusted_scores[candidate_angle],
            )
            visited_table.loc[observed_aspect_bin, selected_wrist_angle] = True
            print(f"Selected wrist angle {selected_wrist_angle} degrees.")

            # Every arm command uses wait=True so each motion finishes before
            # the next one starts. That makes trial timing deterministic and
            # avoids overlapping movements that would confuse the experiment.
            set_wrist_angle(arm_controller, selected_wrist_angle)
            open_gripper(arm_controller)
            move_to_pose(arm_controller, GRASP_POSE, "grasp")
            close_gripper(arm_controller)
            move_to_pose(arm_controller, MID_CARRY_POSE, "mid-carry")

            measured_gripper_position = read_gripper_position(arm_controller)
            gripper_close_error = abs(float(GRIPPER_CLOSED_COUNT) - measured_gripper_position)

            # If measured closure differs from the commanded closed value by
            # more than the threshold, the jaws were likely blocked by the object.
            if gripper_close_error > GRIPPER_BLOCKED_THRESHOLD:
                reward = 1
                trial_result_text = "SUCCESS"
                # Return to grasp position to release the object, then reset.
                move_to_pose(arm_controller, GRASP_POSE_CLOSED, "grasp (release)")
                # [COMMENTED OUT] Optional random wrist re-orientation before release.
                # This only applies when setting a successfully grasped object back down.
                random_release_wrist_angle = float(np.random.uniform(-90.0, 90.0))
                print(f"Random release wrist angle: {random_release_wrist_angle:.1f} degrees")
                set_wrist_angle(arm_controller, random_release_wrist_angle)
                open_gripper(arm_controller)
                move_to_pose(arm_controller, MID_CARRY_POSE, "mid-carry")
            else:
                reward = -1
                trial_result_text = "FAILURE"

            old_q_value = float(q_table.loc[observed_aspect_bin, selected_wrist_angle])

            # The future reward term is omitted here because a grasp attempt is
            # modeled as a single-step decision with an immediate reward only.
            updated_q_value = old_q_value + LEARNING_RATE * reward
            q_table.loc[observed_aspect_bin, selected_wrist_angle] = updated_q_value

            q_table.to_csv(q_table_file)
            print(
                f"Trial result: {trial_result_text}. Updated Q value from {old_q_value:.4f} to {updated_q_value:.4f}."
            )
            print(f"Saved updated Q table to {q_table_file}.")

            result_message = (
                f"Trial result: {trial_result_text}\n"
                f"Observed aspect ratio: {raw_aspect_ratio:.4f}\n"
                f"Object class: {object_class}\n"
                f"Observed bin: {observed_aspect_bin}\n"
                f"Observed-bin count: {observed_bin_counts[float(observed_aspect_bin)]}\n"
                f"Selected wrist angle: {selected_wrist_angle}\n"
                f"Current Q value: {updated_q_value:.4f}"
            )
            if not confirm_or_continue(result_message):
                print("Training stopped by user after the result dialog.")
                raise SystemExit(0)
    finally:
        print("Releasing camera...")
        camera_capture.release()

    print("Training finished.")


if __name__ == "__main__":
    main()
