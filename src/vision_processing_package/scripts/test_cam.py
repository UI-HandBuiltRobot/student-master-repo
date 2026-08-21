import cv2

from robot_config.constants import CAMERA_DEVICE


def open_camera():
    print(f"Opening camera: {CAMERA_DEVICE}")
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera: {CAMERA_DEVICE}")

    print("Camera opened successfully!")
    return cap


def main():
    cap = open_camera()

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to read frame.")
            break

        cv2.imshow("Camera Test", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
