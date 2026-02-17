#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoToGoal(Node):
    def __init__(self):
        super().__init__("go_to_goal")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_auto")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("control_mode_topic", "/control_mode")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("slowdown_radius", 0.35)
        self.declare_parameter("rotate_in_place_angle", 0.5)
        self.declare_parameter("goal_tolerance_xy", 0.08)
        self.declare_parameter("goal_tolerance_yaw", 0.12)
        self.declare_parameter("align_final_heading", True)
        self.declare_parameter("use_external_odom", False)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("auto_switch_mode", True)
        self.declare_parameter("return_to_teleop_on_goal", True)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.goal_topic: str = self.get_parameter(
            "goal_topic"
        ).get_parameter_value().string_value
        self.cmd_vel_topic: str = self.get_parameter(
            "cmd_vel_topic"
        ).get_parameter_value().string_value
        self.odom_topic: str = self.get_parameter(
            "odom_topic"
        ).get_parameter_value().string_value
        self.control_mode_topic: str = self.get_parameter(
            "control_mode_topic"
        ).get_parameter_value().string_value

        self.control_rate_hz: float = self.get_parameter(
            "control_rate_hz"
        ).get_parameter_value().double_value
        self.max_linear_speed: float = self.get_parameter(
            "max_linear_speed"
        ).get_parameter_value().double_value
        self.max_angular_speed: float = self.get_parameter(
            "max_angular_speed"
        ).get_parameter_value().double_value
        self.kp_linear: float = self.get_parameter(
            "kp_linear"
        ).get_parameter_value().double_value
        self.kp_angular: float = self.get_parameter(
            "kp_angular"
        ).get_parameter_value().double_value
        self.slowdown_radius: float = self.get_parameter(
            "slowdown_radius"
        ).get_parameter_value().double_value
        self.rotate_in_place_angle: float = self.get_parameter(
            "rotate_in_place_angle"
        ).get_parameter_value().double_value
        self.goal_tolerance_xy: float = self.get_parameter(
            "goal_tolerance_xy"
        ).get_parameter_value().double_value
        self.goal_tolerance_yaw: float = self.get_parameter(
            "goal_tolerance_yaw"
        ).get_parameter_value().double_value
        self.align_final_heading: bool = self.get_parameter(
            "align_final_heading"
        ).get_parameter_value().bool_value
        self.use_external_odom: bool = self.get_parameter(
            "use_external_odom"
        ).get_parameter_value().bool_value
        self.odom_timeout_sec: float = self.get_parameter(
            "odom_timeout_sec"
        ).get_parameter_value().double_value
        self.auto_switch_mode: bool = self.get_parameter(
            "auto_switch_mode"
        ).get_parameter_value().bool_value
        self.return_to_teleop_on_goal: bool = self.get_parameter(
            "return_to_teleop_on_goal"
        ).get_parameter_value().bool_value
        self.current_x: float = self.get_parameter(
            "initial_x"
        ).get_parameter_value().double_value
        self.current_y: float = self.get_parameter(
            "initial_y"
        ).get_parameter_value().double_value
        self.current_yaw: float = self.get_parameter(
            "initial_yaw"
        ).get_parameter_value().double_value

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_active = False

        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.last_odom_time = None
        self.last_cmd = Twist()
        self.last_control_time = self.wall_clock.now()

        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        if self.use_external_odom:
            self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        else:
            self.odom_sub = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.mode_pub = self.create_publisher(String, self.control_mode_topic, 10)

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.control_loop, clock=self.wall_clock)

        odom_mode = "external /odom" if self.use_external_odom else "internal command integration"
        self.get_logger().info(
            f"GO_TO_GOAL READY | goal={self.goal_topic} cmd_out={self.cmd_vel_topic} "
            f"odom={odom_mode}"
        )

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        quat_norm_sq = qx * qx + qy * qy + qz * qz + qw * qw

        if quat_norm_sq > 1e-8:
            self.goal_yaw = yaw_from_quaternion(qx, qy, qz, qw)
        else:
            self.goal_yaw = self.current_yaw

        self.goal_active = True
        self.get_logger().info(
            f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f}"
        )

        if self.auto_switch_mode:
            self.publish_mode("auto")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.last_odom_time = self.wall_clock.now()

    def publish_mode(self, mode: str):
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)

    def odom_is_fresh(self) -> bool:
        if not self.use_external_odom:
            return True
        if self.last_odom_time is None:
            return False
        odom_age = (self.wall_clock.now() - self.last_odom_time).nanoseconds * 1e-9
        return odom_age <= self.odom_timeout_sec

    def integrate_internal_pose(self, dt: float, cmd: Twist):
        yaw_mid = self.current_yaw + 0.5 * cmd.angular.z * dt
        self.current_x += cmd.linear.x * math.cos(yaw_mid) * dt
        self.current_y += cmd.linear.x * math.sin(yaw_mid) * dt
        self.current_yaw = normalize_angle(self.current_yaw + cmd.angular.z * dt)

    def compute_cmd(self) -> Twist:
        cmd = Twist()

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_yaw)
        yaw_error = normalize_angle(self.goal_yaw - self.current_yaw)

        if distance <= self.goal_tolerance_xy:
            if self.align_final_heading and abs(yaw_error) > self.goal_tolerance_yaw:
                cmd.angular.z = clamp(
                    self.kp_angular * yaw_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
                return cmd

            self.goal_active = False
            self.get_logger().info(
                f"Goal reached at x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}"
            )
            if self.auto_switch_mode and self.return_to_teleop_on_goal:
                self.publish_mode("teleop")
            return cmd

        if abs(heading_error) > self.rotate_in_place_angle:
            linear_speed = 0.0
        else:
            linear_speed = clamp(
                self.kp_linear * distance,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
            linear_speed *= max(0.0, math.cos(heading_error))

            if distance < self.slowdown_radius:
                linear_speed *= distance / max(self.slowdown_radius, 1e-6)

        angular_speed = clamp(
            self.kp_angular * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        return cmd

    def control_loop(self):
        now = self.wall_clock.now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        if dt <= 0.0:
            dt = 1.0 / max(self.control_rate_hz, 1.0)

        cmd = Twist()

        if self.goal_active:
            if not self.odom_is_fresh():
                self.get_logger().warn(
                    "No fresh odometry. Publishing zero cmd_vel until odom arrives.",
                    throttle_duration_sec=2.0,
                )
            else:
                cmd = self.compute_cmd()

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

        if not self.use_external_odom:
            self.integrate_internal_pose(dt, cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
