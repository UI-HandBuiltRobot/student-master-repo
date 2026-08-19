#!/usr/bin/env python3
"""Local tkinter dashboard for the collection-cycle test bench.

One-shot replacement for a full Foxglove setup when you want to sit at
the Jetson and drive the state machine by clicking buttons. Shows:

  - Live camera preview with an overlay for the currently-tracked object:
      * Green rectangle from WSKR/tracked_bbox (width-normalized).
      * Class label inferred from the highest-IoU entry in the latest
        vision/yolo/detections frame (covers toy mode).
      * All detected ArUco markers (yellow) for box-mode visibility.
  - Status strip: robot_state, tracking_mode, fused heading.
  - Button grid that publishes std_msgs/String on /robot_command. The
    state_manager subscribes to /robot_command so any button here is
    equivalent to a Foxglove Publish panel.

Run with:

    ros2 run utilities robot_control_panel

or include it from a launch file (see test_collection.launch.py).
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tkinter as tk
from PIL import Image, ImageTk
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, String

from robot_interfaces.msg import ImgDetectionData, TrackedBbox


IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Buttons are listed top-to-bottom, left-to-right. The first row is the
# quick-access "usual" flow; the second row is the full manual FSM.
BUTTON_ROWS: List[List[Tuple[str, str]]] = [
    [('Idle', 'idle'), ('Wander', 'wander'), ('Stop', 'stop')],
    [('Search Toy', 'search'), ('Select', 'select'), ('Approach Obj', 'approach_obj')],
    [('Grasp', 'grasp'), ('Find Box', 'find_box'), ('Approach Box', 'approach_box')],
    [('Drop', 'drop')],
]

# Max age for bboxes/detections before we stop drawing them.
BBOX_STALENESS_SEC = 0.5
DETECTIONS_STALENESS_SEC = 0.5
ARUCO_STALENESS_SEC = 0.5


def _fit_to_label(frame: np.ndarray, lw: int, lh: int) -> np.ndarray:
    lw = max(lw, 1)
    lh = max(lh, 1)
    sh, sw = frame.shape[:2]
    if sw <= 0 or sh <= 0:
        return np.zeros((lh, lw, 3), dtype=np.uint8)
    scale = min(lw / sw, lh / sh)
    new_w = max(int(sw * scale), 1)
    new_h = max(int(sh * scale), 1)
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((lh, lw, 3), dtype=np.uint8)
    ox = (lw - new_w) // 2
    oy = (lh - new_h) // 2
    canvas[oy:oy + new_h, ox:ox + new_w] = resized
    return canvas


def _bbox_iou(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    iw = max(0.0, x2 - x1)
    ih = max(0.0, y2 - y1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


class ControlPanelNode(Node):
    """ROS-side glue: subscribes to telemetry + publishes /robot_command."""

    def __init__(self) -> None:
        super().__init__('robot_control_panel')

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_t: float = 0.0
        self._tracked_bbox_n: Optional[Tuple[float, float, float, float]] = None
        self._tracked_bbox_t: float = 0.0
        self._latest_detections: Optional[ImgDetectionData] = None
        self._latest_detections_t: float = 0.0
        self._latest_aruco_markers: List[Tuple[int, List[Tuple[float, float]]]] = []
        self._latest_aruco_t: float = 0.0
        self._robot_state = '(unknown)'
        self._tracking_mode = '(unknown)'
        self._heading_deg = 0.0

        self.create_subscription(
            CompressedImage, 'camera1/image_raw/compressed',
            self._on_image, IMAGE_QOS,
        )
        self.create_subscription(
            TrackedBbox, 'WSKR/tracked_bbox',
            self._on_tracked_bbox, 10,
        )
        self.create_subscription(
            Float32MultiArray, 'WSKR/aruco_markers',
            self._on_aruco_markers, 10,
        )
        self.create_subscription(
            ImgDetectionData, 'vision/yolo/detections',
            self._on_detections, IMAGE_QOS,
        )
        self.create_subscription(String, 'robot_state', self._on_state, 10)
        self.create_subscription(String, 'WSKR/tracking_mode', self._on_mode, 10)
        self.create_subscription(Float32, 'WSKR/heading_to_target', self._on_heading, 10)

        self._command_pub = self.create_publisher(String, 'robot_command', 10)

        self.get_logger().info('robot_control_panel ROS node ready.')

    # ------------------------------------------------------------ callbacks
    def _on_image(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            return
        with self._lock:
            self._latest_frame = frame
            self._latest_frame_t = time.time()

    def _on_tracked_bbox(self, msg: TrackedBbox) -> None:
        if not msg.source:
            return
        with self._lock:
            self._tracked_bbox_n = (
                float(msg.x_norm), float(msg.y_norm),
                float(msg.w_norm), float(msg.h_norm),
            )
            self._tracked_bbox_t = time.time()

    def _on_aruco_markers(self, msg: Float32MultiArray) -> None:
        # Layout: flat list of [id, x1n, y1n, ..., x4n, y4n] repeating;
        # each marker uses 9 floats. Coords are width-normalized.
        data = list(msg.data)
        markers: List[Tuple[int, List[Tuple[float, float]]]] = []
        for off in range(0, len(data) - 8, 9):
            marker_id = int(data[off])
            pts = [
                (float(data[off + 1 + 2 * i]), float(data[off + 2 + 2 * i]))
                for i in range(4)
            ]
            markers.append((marker_id, pts))
        with self._lock:
            self._latest_aruco_markers = markers
            self._latest_aruco_t = time.time()

    def _on_detections(self, msg: ImgDetectionData) -> None:
        with self._lock:
            self._latest_detections = msg
            self._latest_detections_t = time.time()

    def _on_state(self, msg: String) -> None:
        with self._lock:
            self._robot_state = msg.data

    def _on_mode(self, msg: String) -> None:
        with self._lock:
            self._tracking_mode = msg.data

    def _on_heading(self, msg: Float32) -> None:
        with self._lock:
            self._heading_deg = float(msg.data)

    # ------------------------------------------------------------- snapshot
    def snapshot(self) -> dict:
        with self._lock:
            return {
                'frame': None if self._latest_frame is None else self._latest_frame.copy(),
                'frame_t': self._latest_frame_t,
                'tracked_bbox_n': self._tracked_bbox_n,
                'tracked_bbox_t': self._tracked_bbox_t,
                'detections': self._latest_detections,
                'detections_t': self._latest_detections_t,
                'aruco': list(self._latest_aruco_markers),
                'aruco_t': self._latest_aruco_t,
                'state': self._robot_state,
                'mode': self._tracking_mode,
                'heading': self._heading_deg,
            }

    # -------------------------------------------------------------- publish
    def publish_command(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._command_pub.publish(msg)
        self.get_logger().info(f'/robot_command <- {cmd!r}')


class ControlPanelUI:
    """Pure-Tk view. Owns the window and the ~15 Hz refresh loop."""

    def __init__(self, node: ControlPanelNode) -> None:
        self._node = node
        self._root = tk.Tk()
        self._root.title('WSKR Robot Control Panel')
        self._root.geometry('960x780')
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)

        # --- status strip ---
        self._status_var = tk.StringVar(value='state: ?   mode: ?   heading: ?')
        status = tk.Label(
            self._root, textvariable=self._status_var, anchor='w',
            font=('TkDefaultFont', 11, 'bold'), padx=8, pady=6,
        )
        status.pack(side='top', fill='x')

        # --- camera preview ---
        self._preview = tk.Label(self._root, bg='black')
        self._preview.pack(side='top', fill='both', expand=True, padx=8, pady=4)

        # --- button grid ---
        btn_frame = tk.Frame(self._root, padx=8, pady=6)
        btn_frame.pack(side='bottom', fill='x')
        for r, row in enumerate(BUTTON_ROWS):
            for c, (label, cmd) in enumerate(row):
                b = tk.Button(
                    btn_frame, text=label, width=14, height=2,
                    command=lambda c=cmd: self._node.publish_command(c),
                )
                b.grid(row=r, column=c, padx=3, pady=3, sticky='ew')
            btn_frame.grid_rowconfigure(r, weight=1)
        for c in range(max(len(r) for r in BUTTON_ROWS)):
            btn_frame.grid_columnconfigure(c, weight=1)

        self._running = True
        self._refresh()

    # ------------------------------------------------------------ lifecycle
    def _on_close(self) -> None:
        self._running = False
        try:
            self._root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self._root.mainloop()

    # ------------------------------------------------------------ rendering
    def _render(self, snap: dict) -> np.ndarray:
        lw = max(self._preview.winfo_width(), 320)
        lh = max(self._preview.winfo_height(), 240)

        frame = snap['frame']
        now = time.time()
        if frame is None or (now - snap['frame_t']) > 2.0:
            canvas = np.zeros((lh, lw, 3), dtype=np.uint8)
            cv2.putText(
                canvas, 'Waiting for camera frames...',
                (20, lh // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (200, 200, 200), 2,
            )
            return canvas

        fh, fw = frame.shape[:2]
        display = frame.copy()

        # Tracked bbox (width-normalized → frame-pixel coords). w/h/y are
        # all normalized by WIDTH (see approach_action_server.py).
        bbox_frame: Optional[Tuple[float, float, float, float]] = None
        if snap['tracked_bbox_n'] is not None and (now - snap['tracked_bbox_t']) <= BBOX_STALENESS_SEC:
            xn, yn, wn, hn = snap['tracked_bbox_n']
            bx = xn * fw
            by = yn * fw
            bw = wn * fw
            bh = hn * fw
            bbox_frame = (bx, by, bw, bh)
            cv2.rectangle(
                display,
                (int(bx), int(by)), (int(bx + bw), int(by + bh)),
                (0, 255, 0), 2,
            )

        # ArUco markers — all drawn in yellow.
        if (now - snap['aruco_t']) <= ARUCO_STALENESS_SEC:
            for marker_id, pts_n in snap['aruco']:
                pts = np.array(
                    [[int(x * fw), int(y * fw)] for (x, y) in pts_n], dtype=np.int32
                )
                cv2.polylines(display, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
                cv2.putText(
                    display, f'ArUco {marker_id}',
                    (int(pts[0, 0]), max(0, int(pts[0, 1]) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                )

        # Class label for the tracked bbox (toy mode). Pick from latest YOLO.
        if bbox_frame is not None and (now - snap['detections_t']) <= DETECTIONS_STALENESS_SEC:
            detections = snap['detections']
            label = self._best_class_for_bbox(bbox_frame, fw, fh, detections)
            if label:
                cv2.putText(
                    display, label,
                    (int(bbox_frame[0]), max(14, int(bbox_frame[1]) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
                )

        return _fit_to_label(display, lw, lh)

    def _best_class_for_bbox(
        self,
        bbox_frame_px: Tuple[float, float, float, float],
        fw: int,
        fh: int,
        detections: Optional[ImgDetectionData],
    ) -> Optional[str]:
        if detections is None or not getattr(detections, 'class_name', None):
            return None
        if not detections.x:
            return None
        dw_det = int(detections.image_width) if detections.image_width else 0
        dh_det = int(detections.image_height) if detections.image_height else 0
        if dw_det <= 0 or dh_det <= 0:
            return None

        # Rescale bbox_frame_px (in live-frame coords) to YOLO's inference
        # space, then compute IoU against each YOLO detection in that space.
        sx = dw_det / float(fw) if fw > 0 else 1.0
        sy = dh_det / float(fh) if fh > 0 else 1.0
        bx, by, bw, bh = bbox_frame_px
        b_in_yolo = (bx * sx, by * sy, bw * sx, bh * sy)

        best_iou = 0.0
        best_label: Optional[str] = None
        n = len(detections.x)
        for i in range(n):
            cx = float(detections.x[i])
            cy = float(detections.y[i])
            ww = float(detections.width[i])
            hh = float(detections.height[i])
            cand = (cx - ww / 2.0, cy - hh / 2.0, ww, hh)
            iou = _bbox_iou(b_in_yolo, cand)
            if iou > best_iou:
                best_iou = iou
                if i < len(detections.class_name):
                    best_label = str(detections.class_name[i])
        if best_iou <= 0.05:
            return None
        return best_label

    # ------------------------------------------------------------ tick loop
    def _refresh(self) -> None:
        if not self._running:
            return
        try:
            snap = self._node.snapshot()
            canvas = self._render(snap)
            rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
            tkimg = ImageTk.PhotoImage(image=Image.fromarray(rgb))
            self._preview.configure(image=tkimg)
            self._preview.image = tkimg  # keep reference

            heading = snap['heading']
            self._status_var.set(
                f"state: {snap['state']}   "
                f"mode: {snap['mode']}   "
                f"heading: {heading:+.1f}°"
            )
        except Exception as exc:
            self._node.get_logger().warn(f'UI refresh failed: {exc}')
        finally:
            self._root.after(66, self._refresh)  # ~15 Hz


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlPanelNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ui = ControlPanelUI(node)
    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
