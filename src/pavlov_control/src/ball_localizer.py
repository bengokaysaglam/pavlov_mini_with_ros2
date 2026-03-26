#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

def rot_y(pitch_rad: float) -> np.ndarray:
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=np.float64,
    )

class BallLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("ball_localizer")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("ball_radius_m", 0.03)

        self.declare_parameter("sat_min", 120)
        self.declare_parameter("val_min", 80)
        self.declare_parameter("hue_low1", 0)
        self.declare_parameter("hue_high1", 10)
        self.declare_parameter("hue_low2", 160)
        self.declare_parameter("hue_high2", 180)

        self.declare_parameter("kernel_size", 5)
        self.declare_parameter("min_radius_px", 3.0)
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("ema_alpha", 0.4)

        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("camera_offset_xyz", [0.04312, 0.0, 0.08341])
        self.declare_parameter("camera_pitch_rad", -0.15)

        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/ball/debug_image")
        self.declare_parameter("ball_point_topic", "/ball/point")
        self.declare_parameter("ball_pose_topic", "/ball/pose")
        self.declare_parameter("ball_visible_topic", "/ball/visible")

        self._bridge = CvBridge()

        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None

        self._last_point_base = None

        self._ball_point_pub = self.create_publisher(PointStamped, self.get_parameter("ball_point_topic").value, 10)
        self._ball_pose_pub = self.create_publisher(PoseStamped, self.get_parameter("ball_pose_topic").value, 10)
        self._ball_visible_pub = self.create_publisher(Bool, self.get_parameter("ball_visible_topic").value, 10)
        self._debug_pub = self.create_publisher(Image, self.get_parameter("debug_image_topic").value, 10)

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "BALL_LOCALIZER READY | "
            f"image={self.get_parameter('image_topic').value} "
            f"info={self.get_parameter('camera_info_topic').value} "
            f"out={self.get_parameter('ball_point_topic').value}"
        )

    # This function's purpose is to receive the camera intrinsics from the CameraInfo topic and store them for later use in the image callback.
    def _camera_info_cb(self, msg: CameraInfo) -> None:
        k = msg.k
        # CameraInfo K matrix layout:
        # [fx  0 cx]
        # [ 0 fy cy]
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        if fx <= 1.0 or fy <= 1.0:
            self.get_logger().warn("CameraInfo intrinsics look invalid (fx/fy <= 1).")
            return

        self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy

    def _publish_visible(self, stamp, visible: bool) -> None:
        msg = Bool()
        msg.data = visible
        self._ball_visible_pub.publish(msg)

    # This function's purpose is to detect the ball in the image and publish its 3D position.
    def _image_cb(self, msg: Image) -> None:
        if self._fx is None:
            self.get_logger().warn("Waiting for /camera/camera_info...", throttle_duration_sec=2.0)
            return

        # This parameters control the behavior of the ball detection and localization. They are read from ROS parameters, which can be set via launch files or command line.
        publish_debug: bool = bool(self.get_parameter("publish_debug_image").value)
        ball_radius_m: float = float(self.get_parameter("ball_radius_m").value)
        min_radius_px: float = float(self.get_parameter("min_radius_px").value)
        max_range_m: float = float(self.get_parameter("max_range_m").value)
        ema_alpha: float = float(self.get_parameter("ema_alpha").value)

        # The following parameters define the HSV color range for detecting the ball. The code supports two hue ranges to handle cases where the hue wraps around (e.g., red color).
        sat_min: int = int(self.get_parameter("sat_min").value)
        val_min: int = int(self.get_parameter("val_min").value)
        hue_low1: int = int(self.get_parameter("hue_low1").value)
        hue_high1: int = int(self.get_parameter("hue_high1").value)
        hue_low2: int = int(self.get_parameter("hue_low2").value)
        hue_high2: int = int(self.get_parameter("hue_high2").value)

        # The kernel_size parameter controls the size of the morphological operations applied to the binary mask. It is ensured to be an odd integer greater than or equal to 1.
        kernel_size: int = int(self.get_parameter("kernel_size").value)
        kernel_size = max(1, kernel_size | 1)

        # 
        output_frame: str = str(self.get_parameter("output_frame").value)
        camera_offset_xyz = self.get_parameter("camera_offset_xyz").value
        camera_pitch_rad: float = float(self.get_parameter("camera_pitch_rad").value)

        # This part converts the incoming ROS Image message to an OpenCV BGR image using cv_bridge. If the conversion fails, it logs an error and returns early.
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return


        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower1 = np.array([hue_low1, sat_min, val_min], dtype=np.uint8)
        upper1 = np.array([hue_high1, 255, 255], dtype=np.uint8)
        lower2 = np.array([hue_low2, sat_min, val_min], dtype=np.uint8)
        upper2 = np.array([hue_high2, 255, 255], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._publish_visible(msg.header.stamp, False)
            if publish_debug:
                self._publish_debug(bgr, mask, None, msg)
            return

        contour = max(contours, key=cv2.contourArea)
        (u, v), radius_px = cv2.minEnclosingCircle(contour)

        if radius_px < min_radius_px:
            self._publish_visible(msg.header.stamp, False)
            if publish_debug:
                self._publish_debug(bgr, mask, (u, v, radius_px), msg, rejected=True)
            return

        fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        f = 0.5 * (fx + fy)
        x_forward = f * ball_radius_m / max(radius_px, 1e-6)

        if x_forward <= 0.0 or x_forward > max_range_m:
            self._publish_visible(msg.header.stamp, False)
            if publish_debug:
                self._publish_debug(bgr, mask, (u, v, radius_px), msg, rejected=True)
            return

        # u = cx + fx * X / Z
        # v = cy + fy * Y / Z
        # => X = (u - cx) * Z / fx
        # => Y = (v - cy) * Z / fy
        y_left = -(u - cx) * x_forward / fx
        z_up = -(v - cy) * x_forward / fy
        p_cam = np.array([x_forward, y_left, z_up], dtype=np.float64)

        if output_frame:
            try:
                t = np.array(
                    [float(camera_offset_xyz[0]), float(camera_offset_xyz[1]), float(camera_offset_xyz[2])],
                    dtype=np.float64,
                )
            except Exception:
                self.get_logger().error(
                    "camera_offset_xyz must be a list like [x, y, z] in meters."
                )
                return
            r = rot_y(camera_pitch_rad)
            p_out = t + r @ p_cam
            frame_id = output_frame
        else:
            p_out = p_cam
            frame_id = msg.header.frame_id or "camera"

        if self._last_point_base is not None and 0.0 < ema_alpha < 1.0:
            p_out = (1.0 - ema_alpha) * self._last_point_base + ema_alpha * p_out
        self._last_point_base = p_out

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame_id
        out.point.x = float(p_out[0])
        out.point.y = float(p_out[1])
        out.point.z = float(p_out[2])
        self._ball_point_pub.publish(out)

        pose = PoseStamped()
        pose.header = out.header
        pose.pose.position = out.point
        pose.pose.orientation.w = 1.0
        self._ball_pose_pub.publish(pose)

        self._publish_visible(msg.header.stamp, True)

        if publish_debug:
            self._publish_debug(bgr, mask, (u, v, radius_px), msg, point=out.point)

    def _publish_debug(self, bgr, mask, circle, msg: Image, rejected: bool = False, point=None) -> None:
        vis = bgr.copy()
        if circle is not None:
            u, v, r = circle
            color = (0, 0, 255) if rejected else (0, 255, 0)
            cv2.circle(vis, (int(u), int(v)), int(r), color, 2)
            cv2.circle(vis, (int(u), int(v)), 3, (255, 255, 255), -1)
            label = f"r={r:.1f}px"
            if point is not None:
                label += f" | x={point.x:.2f} y={point.y:.2f} z={point.z:.2f}"
            cv2.putText(
                vis,
                label,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack([vis, mask_bgr])
        try:
            out = self._bridge.cv2_to_imgmsg(stacked, encoding="bgr8")
        except Exception:
            return
        out.header = msg.header
        self._debug_pub.publish(out)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
