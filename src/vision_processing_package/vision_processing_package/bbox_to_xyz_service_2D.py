#!/usr/bin/env python3

import json
import os
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.srv import BboxToXYZ
from robot_config.constants import XYZ_CONFIG_FILENAME

# Robot datum offsets relative to the calibration datum in millimeters.
X_OFFSET_MM = 300
Y_OFFSET_MM = 130


# Maps frame_orientation_state to a 2x2 matrix M such that
# (user_x, user_y) = M @ (image_cartesian_x, image_cartesian_y), where the
# image Cartesian frame is defined as +X right and +Y up in the raw warp output
# (image_cartesian_x = wx/ppu, image_cartesian_y = -wy/ppu for homography world
# pixel (wx, wy), which has +Y downward).
# Matches data_models._FRAME_ORIENTATION_MATRICES.
_FRAME_ORIENTATION_MATRICES = {
    0: ((+1, 0), (0, +1)),
    1: ((0, +1), (-1, 0)),
    2: ((-1, 0), (0, -1)),
    3: ((0, -1), (+1, 0)),
    4: ((+1, 0), (0, -1)),
    5: ((0, +1), (+1, 0)),
    6: ((-1, 0), (0, +1)),
    7: ((0, -1), (-1, 0)),
}

# Per-state (rotation_op, vflip) for oriented image output.
# Matches corrections._FRAME_DISPLAY_OPS.
_FRAME_DISPLAY_OPS = {
    0: (None, False),
    1: ("cw", False),
    2: ("180", False),
    3: ("ccw", False),
    4: (None, True),
    5: ("cw", True),
    6: ("180", True),
    7: ("ccw", True),
}


def _apply_frame_orientation(image, state):
    """Rotate/flip an image per frame_orientation_state."""
    rot, vflip = _FRAME_DISPLAY_OPS.get(int(state) % 8, (None, False))
    if rot == "cw":
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rot == "ccw":
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rot == "180":
        image = cv2.rotate(image, cv2.ROTATE_180)
    if vflip:
        image = cv2.flip(image, 0)
    return image


# ---------------------------------------------------------------------------
# Scaramuzza omnidirectional model — math ported from py-OCamCalib so the ROS
# node has no runtime dependency on the calibration package.  The forward
# projection (world ray -> source pixel) is world2cam_fast; the inverse
# (source pixel -> 3-D unit ray) is cam2world.  See py-OCamCalib's
# pyocamcalib.modelling.camera for the original implementation.
# ---------------------------------------------------------------------------


def _scaramuzza_world2cam(
    world_points, inverse_poly, distortion_center, stretch_matrix
):
    """3-D world rays -> fisheye image pixels (vectorised)."""
    norms = np.linalg.norm(world_points, axis=1)
    norms_safe = np.where(norms == 0, 1.0, norms)
    z_norm = np.clip(world_points[:, 2] / norms_safe, -1.0, 1.0)
    theta = np.arccos(z_norm)
    rho = np.polyval(inverse_poly, theta)

    pr = np.sqrt(world_points[:, 0] ** 2 + world_points[:, 1] ** 2)
    pr_safe = np.where(pr == 0, np.finfo(float).eps, pr)
    px = (world_points[:, 0] / pr_safe) * rho
    py = (world_points[:, 1] / pr_safe) * rho

    # Replicates py-OCamCalib's (slightly buggy) ordering — second line uses
    # the already-modified px — for behavioural parity with the GUI's
    # _build_scaramuzza_maps.  Difference vs the strictly correct formula is
    # negligible because stretch_matrix is near-identity.
    px_new = px * stretch_matrix[0, 0] + py * stretch_matrix[0, 1]
    py_new = px_new * stretch_matrix[1, 0] + py
    return np.column_stack(
        (px_new + distortion_center[0], py_new + distortion_center[1])
    )


def _scaramuzza_cam2world(
    uv_points, taylor_coefficient, distortion_center, stretch_matrix
):
    """Fisheye image pixels -> unit 3-D rays."""
    uv = uv_points - np.asarray(distortion_center, dtype=np.float64)
    uv = uv @ np.linalg.inv(stretch_matrix).T
    rho = np.sqrt(uv[:, 0] ** 2 + uv[:, 1] ** 2)
    # taylor_coefficient is stored ascending [a0, 0, a2, a3, a4]; np.polyval
    # wants highest-degree-first, so reverse.
    z = np.polyval(np.asarray(taylor_coefficient)[::-1], rho)
    norm = np.sqrt(uv[:, 0] ** 2 + uv[:, 1] ** 2 + z**2)
    return np.column_stack((uv[:, 0] / norm, uv[:, 1] / norm, z / norm))


def _build_scaramuzza_maps(params, fov_deg, out_w, out_h):
    """cv2.remap source-pixel maps for a virtual perspective camera at fov_deg."""
    inverse_poly = np.asarray(params["inverse_poly"], dtype=np.float64)
    dc = params["distortion_center"]
    stretch = np.asarray(params["stretch_matrix"], dtype=np.float64)

    fov_deg = min(float(fov_deg), 179.0)  # tan(90°) → ∞ otherwise
    f = max(out_w, out_h) / (2.0 * np.tan(np.radians(fov_deg / 2.0)))

    u = np.arange(out_w, dtype=np.float64) - out_w / 2.0
    v = np.arange(out_h, dtype=np.float64) - out_h / 2.0
    X, Y = np.meshgrid(u, v)
    world_points = np.column_stack(
        (X.ravel(), Y.ravel(), np.full(X.size, f, dtype=np.float64))
    )

    img_pts = _scaramuzza_world2cam(world_points, inverse_poly, dc, stretch)
    mapx = img_pts[:, 0].reshape(out_h, out_w).astype(np.float32)
    mapy = img_pts[:, 1].reshape(out_h, out_w).astype(np.float32)
    return mapx, mapy


class BboxToXYZServiceNode(Node):
    def __init__(self):
        """Register the endpoint and load camera calibration for 2D point mapping."""
        super().__init__("bbox_to_xyz_node")

        # Get calibration path from launch parameter or use default from constants
        calibration_path = self.declare_parameter('calibration_path', '').value
        self._load_calibration(calibration_path)

        self.create_service(BboxToXYZ, "bbox_to_xyz_service", self._handle_request)

        self._bridge = CvBridge()
        self._img_pub = self.create_publisher(Image, "img_corrected", 10)
        self.create_subscription(Image, "img_raw", self._img_raw_callback, 10)

        self.get_logger().info("BboxToXYZ Service ready (2D calibrated mapping).")

    def _load_calibration(self, calibration_path):
        """Load intrinsics, distortion, perspective transform, and scaling from JSON."""
        with open(calibration_path, "r", encoding="utf-8") as file:
            calibration = json.load(file)

        intrinsics = calibration.get("intrinsics", {})
        extrinsics = calibration.get("extrinsics", {})
        scaling = calibration.get("scaling", {})
        calibration_info = calibration.get("calibration_info", {})

        image_size = calibration_info.get("image_size", [1920, 1080])
        if len(image_size) == 2 and image_size[0] > 0 and image_size[1] > 0:
            self.default_image_width = float(image_size[0])
            self.default_image_height = float(image_size[1])
        else:
            self.default_image_width = 1920.0
            self.default_image_height = 1080.0

        # Lens model: pinhole/rational use cv2.undistort; fisheye uses
        # cv2.fisheye.undistortImage; scaramuzza uses our ported math.
        self.model_type = str(intrinsics.get("model_type", "pinhole")).lower()

        cm_raw = intrinsics.get("camera_matrix")
        self.camera_matrix = (
            np.asarray(cm_raw, dtype=np.float64) if cm_raw is not None else None
        )
        dist_raw = intrinsics.get("distortion_coefficients")
        if dist_raw is not None:
            distortion = np.asarray(dist_raw, dtype=np.float64)
            if distortion.ndim == 2 and distortion.shape[0] == 1:
                distortion = distortion[0]
            self.distortion_coeffs = distortion.reshape(-1, 1)
        else:
            self.distortion_coeffs = None

        # Fisheye-only.
        self.fisheye_balance = float(intrinsics.get("fisheye_balance", 1.0))
        self.fisheye_Knew = None  # filled below if needed

        # Scaramuzza-only.  scaramuzza_params is the dict produced by the GUI
        # (taylor_coefficient, inverse_poly, distortion_center, stretch_matrix).
        self.scaramuzza_params = intrinsics.get("scaramuzza_params")
        self.scaramuzza_fov = float(intrinsics.get("scaramuzza_fov", 150.0))
        self._scaramuzza_remap_cache = None  # (w, h) -> (mapx, mapy)

        self.perspective_matrix = np.asarray(
            extrinsics.get("perspective_matrix", []), dtype=np.float64
        )

        # Output bbox (x_min, y_min, x_max, y_max) in world coords at native
        # homography scale.  Used to crop and size the oriented image output.
        bbox = extrinsics.get("output_bbox_world")
        self.output_bbox_world = (
            tuple(float(v) for v in bbox) if bbox is not None else None
        )

        # Coordinate-frame orientation state (0-7).  Drives both the 2x2
        # matrix used in point conversion and the rotation/flip applied to
        # the published corrected image.
        self.frame_orientation_state = (
            int(extrinsics.get("frame_orientation_state", 4)) % 8
        )

        # Downsample factor baked into the physical canvas at calibration time.
        self.output_scale = float(scaling.get("output_scale", 1.0))

        # Point conversion must use the native (pre-downsample) pixels_per_real_unit
        # because cv2.perspectiveTransform outputs world coords at the raw
        # homography scale, not the downsampled physical canvas scale.
        # Fall back to the effective value for older exports that lack _native.
        ppu_native = scaling.get("pixels_per_real_unit_native")
        if ppu_native and float(ppu_native) > 0.0:
            self.pixels_per_mm_native = float(ppu_native)
        else:
            ppu_eff = scaling.get("pixels_per_real_unit", 0.0)
            if ppu_eff and float(ppu_eff) > 0.0:
                self.pixels_per_mm_native = float(ppu_eff)
            else:
                sq_px = float(scaling.get("square_size_pixels", 0.0))
                sq_real = float(scaling.get("square_size_real", 0.0))
                if sq_real <= 0.0 or sq_px <= 0.0:
                    raise ValueError("Invalid scaling in camera_calibration.json")
                self.pixels_per_mm_native = sq_px / sq_real

        if self.perspective_matrix.shape != (3, 3):
            raise ValueError(
                "perspective_matrix must be 3x3 in camera_calibration.json"
            )
        if self.pixels_per_mm_native <= 0.0:
            raise ValueError(
                "pixels_per_mm must be positive in camera_calibration.json"
            )

        if self.model_type in ("pinhole", "rational"):
            if self.camera_matrix is None or self.camera_matrix.shape != (3, 3):
                raise ValueError(
                    f"camera_matrix must be 3x3 for model_type='{self.model_type}'"
                )
        elif self.model_type == "fisheye":
            if self.camera_matrix is None or self.camera_matrix.shape != (3, 3):
                raise ValueError("camera_matrix must be 3x3 for fisheye")
            # Knew controls how much of the source FOV the undistorted image
            # preserves.  Matches the GUI's _apply_lens fisheye branch.
            W, H = int(self.default_image_width), int(self.default_image_height)
            self.fisheye_Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.camera_matrix,
                self.distortion_coeffs,
                (W, H),
                np.eye(3),
                balance=self.fisheye_balance,
            )
        elif self.model_type == "scaramuzza":
            if not self.scaramuzza_params:
                raise ValueError(
                    "scaramuzza_params required for model_type='scaramuzza'"
                )
            for key in (
                "taylor_coefficient",
                "inverse_poly",
                "distortion_center",
                "stretch_matrix",
            ):
                if key not in self.scaramuzza_params:
                    raise ValueError(f"scaramuzza_params missing '{key}'")
        else:
            raise ValueError(f"Unsupported model_type='{self.model_type}'")

    def _undistort_image(self, frame):
        """Apply lens correction per loaded model_type. Output is same size as frame."""
        if self.model_type in ("pinhole", "rational"):
            return cv2.undistort(frame, self.camera_matrix, self.distortion_coeffs)
        if self.model_type == "fisheye":
            return cv2.fisheye.undistortImage(
                frame,
                self.camera_matrix,
                D=self.distortion_coeffs,
                Knew=self.fisheye_Knew,
                new_size=(frame.shape[1], frame.shape[0]),
            )
        # Scaramuzza
        h, w = frame.shape[:2]
        cache = self._scaramuzza_remap_cache
        if cache is None or cache[0] != (w, h):
            mapx, mapy = _build_scaramuzza_maps(
                self.scaramuzza_params, self.scaramuzza_fov, w, h
            )
            self._scaramuzza_remap_cache = ((w, h), mapx, mapy)
        else:
            _, mapx, mapy = cache
        return cv2.remap(
            frame,
            mapx,
            mapy,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

    def _img_raw_callback(self, msg: Image):
        """Undistort and perspective-correct an incoming raw image, then republish."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        h, w = frame.shape[:2]
        undistorted = self._undistort_image(frame)

        if self.output_bbox_world is not None:
            x_min, y_min, x_max, y_max = self.output_bbox_world
            s_inv = 1.0 / self.output_scale
            out_w = max(1, int(np.ceil((x_max - x_min) * s_inv)))
            out_h = max(1, int(np.ceil((y_max - y_min) * s_inv)))
            # Fold bbox translation and downsampling into a single homography so
            # the warp produces the same pixel layout as the tracking-app display.
            M_combined = (
                np.array(
                    [
                        [s_inv, 0.0, -x_min * s_inv],
                        [0.0, s_inv, -y_min * s_inv],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=np.float64,
                )
                @ self.perspective_matrix
            )
            warped = cv2.warpPerspective(
                undistorted,
                M_combined,
                (out_w, out_h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )
            corrected = _apply_frame_orientation(warped, self.frame_orientation_state)
        else:
            # No bbox stored (older calibration export) — fall back to full-frame warp.
            corrected = cv2.warpPerspective(
                undistorted, self.perspective_matrix, (w, h)
            )

        out_msg = self._bridge.cv2_to_imgmsg(corrected, encoding="bgr8")
        out_msg.header = msg.header
        self._img_pub.publish(out_msg)

    def _handle_request(self, request, response):
        """Map bbox center pixel -> undistort -> perspective-correct -> user-frame mm coordinates."""
        bbox_x = float(request.bbox_x)
        bbox_y = float(request.bbox_y)

        if request.image_width > 0 and request.image_height > 0:
            image_width = float(request.image_width)
            image_height = float(request.image_height)

            # Handle callers that still send normalized coordinates with explicit image dimensions.
            if 0.0 <= bbox_x <= 1.0 and 0.0 <= bbox_y <= 1.0:
                pixel_x = bbox_x * image_width
                pixel_y = bbox_y * image_height
            else:
                pixel_x = bbox_x
                pixel_y = bbox_y
        else:
            # If dimensions are omitted, treat request coordinates as normalized [0, 1].
            pixel_x = bbox_x * self.default_image_width
            pixel_y = bbox_y * self.default_image_height

        if pixel_x < 0.0 or pixel_y < 0.0:
            self.get_logger().error(
                f"Invalid bbox center pixel ({pixel_x:.3f}, {pixel_y:.3f}); cannot compute 2D position."
            )
            response.success = False
            response.x_mm = 0.0
            response.y_mm = 0.0
            response.z_mm = 0.0
            return response

        input_points = np.array([[[pixel_x, pixel_y]]], dtype=np.float64)

        if self.model_type in ("pinhole", "rational"):
            undistorted = cv2.undistortPoints(
                src=input_points,
                cameraMatrix=self.camera_matrix,
                distCoeffs=self.distortion_coeffs,
                P=self.camera_matrix,
            )
        elif self.model_type == "fisheye":
            undistorted = cv2.fisheye.undistortPoints(
                input_points,
                K=self.camera_matrix,
                D=self.distortion_coeffs,
                P=self.fisheye_Knew,
            )
        else:  # scaramuzza
            # Raw fisheye pixel -> 3-D unit ray -> project through the same
            # virtual perspective camera used to build the image-undistort
            # maps.  The resulting pixel lies in the canvas the homography
            # was calibrated against.
            ray = _scaramuzza_cam2world(
                np.array([[pixel_x, pixel_y]], dtype=np.float64),
                np.asarray(
                    self.scaramuzza_params["taylor_coefficient"], dtype=np.float64
                ),
                self.scaramuzza_params["distortion_center"],
                np.asarray(self.scaramuzza_params["stretch_matrix"], dtype=np.float64),
            )[0]
            if ray[2] <= 0.0:
                self.get_logger().error(
                    f"Pixel ({pixel_x:.1f}, {pixel_y:.1f}) projects behind "
                    "the virtual perspective camera; cannot map."
                )
                response.success = False
                response.x_mm = 0.0
                response.y_mm = 0.0
                response.z_mm = 0.0
                return response
            W = self.default_image_width
            H = self.default_image_height
            fov = min(self.scaramuzza_fov, 179.0)
            f_persp = max(W, H) / (2.0 * np.tan(np.radians(fov / 2.0)))
            persp_x = ray[0] * f_persp / ray[2] + W / 2.0
            persp_y = ray[1] * f_persp / ray[2] + H / 2.0
            undistorted = np.array([[[persp_x, persp_y]]], dtype=np.float64)

        world_pts = cv2.perspectiveTransform(undistorted, self.perspective_matrix)

        wx = float(world_pts[0, 0, 0])
        wy = float(world_pts[0, 0, 1])

        # The homography Y axis is downward (+wy = down in the physical plane).
        # The frame-orientation matrix expects image Cartesian coords where +Y is up,
        # so we negate wy before applying M.  The resulting (x_user, y_user) are in
        # the coordinate frame chosen in the tracking app (e.g. X right / Y up, or
        # X up / Y left, etc.) with units of mm at native homography scale.
        m = _FRAME_ORIENTATION_MATRICES[self.frame_orientation_state]
        x_cart = wx / self.pixels_per_mm_native
        y_cart = -wy / self.pixels_per_mm_native
        x_user_mm = m[0][0] * x_cart + m[0][1] * y_cart
        y_user_mm = m[1][0] * x_cart + m[1][1] * y_cart

        # Shift from calibration datum into robot datum coordinates.
        final_x_mm = x_user_mm + X_OFFSET_MM
        final_y_mm = y_user_mm + Y_OFFSET_MM

        self.get_logger().info(
            "BboxToXYZ 2D: "
            f"pixel=({pixel_x:.1f}, {pixel_y:.1f}) -> "
            f"world_px=({wx:.1f}, {wy:.1f}) -> "
            f"user_mm=({x_user_mm:.1f}, {y_user_mm:.1f}) -> "
            f"robot_mm=({final_x_mm:.1f}, {final_y_mm:.1f})"
        )

        response.success = True
        response.x_mm = float(final_x_mm)
        response.y_mm = float(final_y_mm)
        response.z_mm = 0.0

        return response


def main(args=None):
    """Run the BboxToXYZ node until interrupted, then shut ROS down cleanly."""
    rclpy.init(args=args)
    node = BboxToXYZServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
