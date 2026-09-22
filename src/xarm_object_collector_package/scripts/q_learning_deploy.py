"""Deploy a trained Q table without updating it.

Purpose:
- Observe an object, choose the best learned wrist angle for the nearest state,
  attempt a grasp, and report whether it succeeded.

Inputs:
- A trained Q table from constants.Q_TABLE_FILE.
- Camera images from constants.CAMERA_DEVICE.
- Roboflow detections from inference_sdk using the shared model constants.

Outputs:
- Terminal status messages and tkinter result dialogs.

Dependencies:
- xarm
- inference_sdk
- cv2
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
True
from robot_config.constants import (
    ASPECT_BINS,
    CAMERA_DEVICE,
    ENABLE_IMG_DEG_ROTATION_SIGN_FEATURE,
    Q_ARM_SERVO_IDS,
    GRASP_POSE,
    GRASP_POSE_CLOSED,
    GRIPPER_BLOCKED_THRESHOLD,
    GRIPPER_CLOSED_COUNT,
    GRIPPER_OPEN_COUNT,
    GRIPPER_SERVO_INDEX,
    MID_CARRY_POSE,
    Q_TABLE_FILE,
    ROBOFLOW_API_KEY,
    ROBOFLOW_MODEL_ID,
    ROBOFLOW_VERSION,
    TRAIN_PER_CLASS,
    WRIST_ANGLES,
    WRIST_SERVO_ID,
    IMG_ROTATION_ANGLE,
)

POSE_EXCLUDED_SERVO_IDS = {1, 2}


def build_state_bins():
    """Return the state bins, optionally mirrored to include signed bins."""
    base_bins = [abs(float(aspect_bin)) for aspect_bin in ASPECT_BINS]
    base_bins = list(dict.fromkeys(base_bins))

    if not ENABLE_IMG_DEG_ROTATION_SIGN_FEATURE:
        return base_bins

    mirrored_bins = list(base_bins)
    mirrored_bins.extend([-aspect_bin for aspect_bin in base_bins if aspect_bin != 0.0])
    return mirrored_bins


def show_ok_or_exit_dialog(message_text, window_title="Q-Learning Deploy"):
    """Show a dialog with OK to continue. Closing the window means exit."""
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

    root_window.protocol("WM_DELETE_WINDOW", root_window.destroy)
    root_window.mainloop()
    return dialog_result["ok_clicked"]


def confirm_or_continue(message_text, window_title="Q-Learning Deploy"):
    """Always show the deploy dialog and continue only when user clicks OK."""
    return show_ok_or_exit_dialog(message_text, window_title=window_title)


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
    """Return the generic or class-specific Q-table file path."""
    if not TRAIN_PER_CLASS or object_class is None:
        return Q_TABLE_FILE

    file_root, file_extension = os.path.splitext(Q_TABLE_FILE)
    return f"{file_root}_{sanitize_object_class_name(object_class)}{file_extension or '.csv'}"


def load_q_table():
    """Load the trained Q table, or exit if it does not exist."""
    if not os.path.exists(Q_TABLE_FILE):
        print(f"Error: trained Q table not found at {Q_TABLE_FILE}.")
        raise SystemExit(1)

    print(f"Loading trained Q table from {Q_TABLE_FILE}...")
    q_table = pd.read_csv(Q_TABLE_FILE, index_col=0)
    q_table.index = q_table.index.astype(float)
    q_table.columns = q_table.columns.astype(float)
    print("Loaded Q table.")
    return q_table


def load_or_initialize_class_q_table(object_class):
    """Load or create a class-specific Q table using the current training schema."""
    q_table_file = build_q_table_file_path(object_class)
    expected_index = pd.Index(build_state_bins(), dtype=float)
    expected_columns = pd.Index([float(wrist_angle) for wrist_angle in WRIST_ANGLES], dtype=float)

    def create_new_q_table():
        print(f"Creating a new Q table at {q_table_file}...")
        values = np.random.uniform(0.0, 0.01, size=(len(expected_index), len(expected_columns)))
        q_table = pd.DataFrame(values, index=expected_index, columns=expected_columns)
        q_table.to_csv(q_table_file)
        print("Created new Q table.")
        return q_table

    if not os.path.exists(q_table_file):
        return q_table_file, create_new_q_table()

    if os.path.getsize(q_table_file) == 0:
        print(f"Q table file {q_table_file} is empty. Reinitializing.")
        return q_table_file, create_new_q_table()

    print(f"Loading trained Q table from {q_table_file}...")
    try:
        q_table = pd.read_csv(q_table_file, index_col=0)
    except pd.errors.EmptyDataError:
        print(f"Q table file {q_table_file} has no CSV data. Reinitializing.")
        return q_table_file, create_new_q_table()

    if q_table.empty:
        print(f"Q table file {q_table_file} has no rows. Reinitializing.")
        return q_table_file, create_new_q_table()

    q_table.index = q_table.index.astype(float)
    q_table.columns = q_table.columns.astype(float)
    if q_table.index.equals(expected_index) and q_table.columns.equals(expected_columns):
        print("Loaded Q table.")
        return q_table_file, q_table

    print("Existing Q table schema does not match current constants. Reindexing table...")
    q_table = q_table.reindex(index=expected_index, columns=expected_columns)
    missing_mask = q_table.isna()
    if missing_mask.any().any():
        fill = pd.DataFrame(
            np.random.uniform(0.0, 0.01, size=q_table.shape),
            index=q_table.index,
            columns=q_table.columns,
        )
        q_table = q_table.where(~missing_mask, fill)
    q_table.to_csv(q_table_file)
    print("Saved reindexed Q table to match current constants.")
    print("Loaded Q table.")
    return q_table_file, q_table


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
    """Open the configured USB camera."""
    print(f"Opening USB camera at index {CAMERA_DEVICE}...")
    camera_capture = cv2.VideoCapture(CAMERA_DEVICE)
    if not camera_capture.isOpened():
        raise RuntimeError(f"Could not open camera index {CAMERA_DEVICE}.")
    print("Camera opened.")
    return camera_capture


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
    """Return the highest-confidence detection from the Roboflow response."""
    predictions = prediction_json.get("predictions", [])
    if not predictions:
        return None
    return max(predictions, key=lambda prediction: prediction.get("confidence", 0.0))


def flush_camera_buffer(camera_capture, num_frames=5):
    """Discard buffered frames so read() returns a fresh image."""
    for _ in range(num_frames):
        camera_capture.grab()


def show_image_with_bounding_box(frame_bgr, detection, window_name="Object Detection Preview", ar_label=None):
    """Display one preview image with optional detection box and aspect-ratio label."""
    frame_with_box = frame_bgr.copy()

    if detection is not None:
        x = float(detection.get("x", 0))
        y = float(detection.get("y", 0))
        width = float(detection.get("width", 0))
        height = float(detection.get("height", 0))
        confidence = float(detection.get("confidence", 0.0))

        left = int(x - width / 2)
        top = int(y - height / 2)
        right = int(x + width / 2)
        bottom = int(y + height / 2)
        cv2.rectangle(frame_with_box, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(
            frame_with_box,
            f"Confidence: {confidence:.2f}",
            (left, top - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        observed_ar = width / height if height > 0 else 0.0
        label = ar_label or f"Observed aspect ratio: {observed_ar:.4f}"
    else:
        cv2.putText(frame_with_box, "No object detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        label = ar_label or "Observed aspect ratio: N/A"

    cv2.putText(frame_with_box, label, (20, frame_with_box.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.imshow(window_name, frame_with_box)
    for _ in range(10):
        cv2.waitKey(30)


def infer_highest_confidence_detection_from_frame(frame_bgr, inference_client):
    """Run inference on a frame and return the highest-confidence detection or None."""
    temporary_image_path = save_frame_to_temporary_image(frame_bgr)
    try:
        prediction_json = run_inference(inference_client, temporary_image_path)
    finally:
        if os.path.exists(temporary_image_path):
            os.remove(temporary_image_path)
    return extract_highest_confidence_detection(prediction_json)


def capture_and_infer(camera_capture, inference_client, available_bins):
    """Capture frame, infer AR, and map to nearest trained bin (signed if enabled)."""
    flush_camera_buffer(camera_capture)
    frame_was_captured, frame_bgr = camera_capture.read()
    if not frame_was_captured:
        raise RuntimeError("Failed to capture a frame from the USB camera.")

    best_detection = infer_highest_confidence_detection_from_frame(frame_bgr, inference_client)
    show_image_with_bounding_box(frame_bgr, best_detection)
    if best_detection is None:
        raise RuntimeError("No object was detected in the camera image.")

    bounding_box_width = float(best_detection["width"])
    bounding_box_height = float(best_detection["height"])
    if bounding_box_height <= 0.0:
        raise RuntimeError("The detected bounding box height was zero or negative.")

    raw_aspect_ratio = bounding_box_width / bounding_box_height
    object_class = extract_object_class_name(best_detection)

    absolute_trained_bins = sorted({abs(float(candidate_bin)) for candidate_bin in available_bins})
    nearest_abs_bin = min(absolute_trained_bins, key=lambda candidate_bin: abs(candidate_bin - raw_aspect_ratio))
    signed_target_bin = nearest_abs_bin

    if ENABLE_IMG_DEG_ROTATION_SIGN_FEATURE:
        rotation_center = (frame_bgr.shape[1] / 2.0, frame_bgr.shape[0] / 2.0)
        rotation_matrix = cv2.getRotationMatrix2D(rotation_center, IMG_ROTATION_ANGLE, 1.0)
        rotated_frame = cv2.warpAffine(
            frame_bgr,
            rotation_matrix,
            (frame_bgr.shape[1], frame_bgr.shape[0]),
        )
        rotated_detection = infer_highest_confidence_detection_from_frame(rotated_frame, inference_client)
        if rotated_detection is None:
            show_image_with_bounding_box(
                rotated_frame,
                rotated_detection,
                window_name=f"Object Detection Preview ({IMG_ROTATION_ANGLE} deg)",
                ar_label="Signed aspect ratio: N/A",
            )
            raise RuntimeError(f"No object was detected in the {IMG_ROTATION_ANGLE}-degree rotated image.")

        rotated_width = float(rotated_detection["width"])
        rotated_height = float(rotated_detection["height"])
        if rotated_height <= 0.0:
            raise RuntimeError("The rotated detected bounding box height was zero or negative.")

        rotated_aspect_ratio = rotated_width / rotated_height
        signed_target_bin = nearest_abs_bin if (rotated_aspect_ratio - raw_aspect_ratio) >= 0.0 else -nearest_abs_bin
        signed_ar = raw_aspect_ratio if signed_target_bin >= 0.0 else -raw_aspect_ratio
        show_image_with_bounding_box(
            rotated_frame,
            rotated_detection,
            window_name=f"Object Detection Preview ({IMG_ROTATION_ANGLE} deg)",
            ar_label=f"Signed aspect ratio: {signed_ar:.4f}",
        )

    nearest_aspect_bin = min(available_bins, key=lambda candidate_bin: abs(float(candidate_bin) - signed_target_bin))
    return raw_aspect_ratio, float(nearest_aspect_bin), object_class


def estimate_bin_width(sorted_bins):
    """Estimate the typical spacing between neighboring trained bins."""
    if len(sorted_bins) < 2:
        return 0.0

    neighbor_spacings = []
    for index in range(1, len(sorted_bins)):
        neighbor_spacings.append(float(sorted_bins[index]) - float(sorted_bins[index - 1]))
    return sum(neighbor_spacings) / len(neighbor_spacings)


def pose_to_servo_commands(pose_positions):
    """Convert a saved pose list into the servo/value structure expected by xarm."""
    return [
        [servo_id, pose_positions[index]]
        for index, servo_id in enumerate(Q_ARM_SERVO_IDS)
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
    print("Starting Q-learning deployment.")

    q_table_cache = {}
    if not TRAIN_PER_CLASS:
        q_table_cache[Q_TABLE_FILE] = load_q_table()

    arm_controller = connect_to_arm()
    inference_client = connect_to_inference_client()
    camera_capture = open_camera()

    try:
        while True:
            if not confirm_or_continue("Click OK to attempt a grasp."):
                print("Deployment stopped by user.")
                break

            while True:
                cv2.destroyAllWindows()
                print("Capturing image and running inference...")
                try:
                    if TRAIN_PER_CLASS:
                        raw_aspect_ratio, observed_aspect_bin, object_class = capture_and_infer(
                            camera_capture,
                            inference_client,
                            build_state_bins(),
                        )
                        q_table_file, q_table = load_or_initialize_class_q_table(object_class)
                        q_table_cache[q_table_file] = q_table
                    else:
                        q_table = q_table_cache[Q_TABLE_FILE]
                        q_table_file = Q_TABLE_FILE
                        raw_aspect_ratio, observed_aspect_bin, object_class = capture_and_infer(
                            camera_capture,
                            inference_client,
                            sorted(float(bin_value) for bin_value in q_table.index.tolist()),
                        )
                    break
                except RuntimeError as inference_error:
                    print(f"Inference failed: {inference_error}")
                    if not confirm_or_continue(
                        f"Inference failed. Reposition the object and try again.\n\n({inference_error})"
                    ):
                        print("Deployment stopped by user.")
                        return

            selected_wrist_angle = float(q_table.loc[observed_aspect_bin].idxmax())
            selected_q_value = float(q_table.loc[observed_aspect_bin, selected_wrist_angle])
            print(
                f"Observed aspect ratio {raw_aspect_ratio:.4f}, mapped bin {observed_aspect_bin}, "
                f"selected wrist angle {selected_wrist_angle}, Q value {selected_q_value:.4f}."
            )
            if TRAIN_PER_CLASS:
                print(f"Detected object class '{object_class}', using Q table {q_table_file}.")

            set_wrist_angle(arm_controller, selected_wrist_angle)
            open_gripper(arm_controller)
            move_to_pose(arm_controller, GRASP_POSE, "grasp")
            close_gripper(arm_controller)
            move_to_pose(arm_controller, MID_CARRY_POSE, "mid-carry")

            measured_gripper_position = read_gripper_position(arm_controller)
            gripper_close_error = abs(float(GRIPPER_CLOSED_COUNT) - measured_gripper_position)
            if gripper_close_error > float(GRIPPER_BLOCKED_THRESHOLD):
                grasp_result_text = "SUCCESS"
                # Successful pickup handling: release deterministically with no
                # random wrist rotation.
                move_to_pose(arm_controller, GRASP_POSE_CLOSED, "grasp (release)")
                open_gripper(arm_controller)
                move_to_pose(arm_controller, MID_CARRY_POSE, "mid-carry")
            else:
                grasp_result_text = "FAILURE"

            result_message = (
                f"Result: {grasp_result_text}\n"
                f"Observed aspect ratio: {raw_aspect_ratio:.4f}\n"
                f"Object class: {object_class}\n"
                f"Used bin: {observed_aspect_bin}\n"
                f"Selected wrist angle: {selected_wrist_angle}\n"
                f"Q value used: {selected_q_value:.4f}\n\n"
                "Click OK to attempt another grasp, or close the window to exit."
            )
            if not confirm_or_continue(result_message):
                print("Deployment stopped by user.")
                break
    finally:
        print("Releasing camera...")
        camera_capture.release()

    print("Deployment finished.")


if __name__ == "__main__":
    main()
