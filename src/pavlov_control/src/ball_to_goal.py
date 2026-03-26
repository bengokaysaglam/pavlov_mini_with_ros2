#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

class BallToGoal(Node):
    def __init__(self) -> None:
        super().__init__("ball_to_goal")

        self.declare_parameter("ball_point_topic", "/ball/point")
        self.declare_parameter("ball_visible_topic", "/ball/visible")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("control_mode_topic", "/control_mode")

        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("visible_timeout_sec", 0.6)

        self.declare_parameter("standoff_m", 0.25)
        self.declare_parameter("min_ball_x_m", 0.15)
        self.declare_parameter("max_goal_x_m", 6.0)
        self.declare_parameter("max_goal_y_m", 2.0)
        self.declare_parameter("goal_frame", "base_link")

        self.declare_parameter("force_auto_mode", True)
        self.declare_parameter("switch_to_teleop_on_lost", True)

        # Use wall/system time for timers and freshness checks so this node still works
        # even if /clock is not available or sim time is paused.
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self._ball_visible = False
        self._last_visible_time = None

        self._last_ball_point = None
        self._last_ball_time = None

        self._lost_announced = True

        self._goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
        self._mode_pub = self.create_publisher(String, self.get_parameter("control_mode_topic").value, 10)

        self.create_subscription(
            Bool,
            self.get_parameter("ball_visible_topic").value,
            self._visible_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("ball_point_topic").value,
            self._ball_point_cb,
            qos_profile_sensor_data,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(
            1.0 / max(publish_rate_hz, 0.5), self._tick, clock=self._wall_clock
        )

        self.get_logger().info(
            "BALL_TO_GOAL READY | "
            f"ball={self.get_parameter('ball_point_topic').value} "
            f"visible={self.get_parameter('ball_visible_topic').value} "
            f"goal_out={self.get_parameter('goal_topic').value}"
        )

    def _visible_cb(self, msg: Bool) -> None:
        self._ball_visible = bool(msg.data)
        self._last_visible_time = self._wall_clock.now()

    def _ball_point_cb(self, msg: PointStamped) -> None:
        self._last_ball_point = msg
        self._last_ball_time = self._wall_clock.now()

    def _publish_mode(self, mode: str) -> None:
        m = String()
        m.data = mode
        self._mode_pub.publish(m)

    def _is_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self._wall_clock.now() - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    def _tick(self) -> None:
        visible_timeout_sec = float(self.get_parameter("visible_timeout_sec").value)
        force_auto_mode = bool(self.get_parameter("force_auto_mode").value)
        switch_to_teleop_on_lost = bool(self.get_parameter("switch_to_teleop_on_lost").value)

        ball_fresh = self._is_fresh(self._last_ball_time, visible_timeout_sec)
        visible_fresh = self._is_fresh(self._last_visible_time, visible_timeout_sec)

        if not (ball_fresh and visible_fresh and self._ball_visible and self._last_ball_point is not None):
            if switch_to_teleop_on_lost and not self._lost_announced:
                self._publish_mode("teleop")
                self.get_logger().warn("Ball lost -> switching to teleop", throttle_duration_sec=2.0)
            self._lost_announced = True
            return

        p = self._last_ball_point.point
        ball_x = float(p.x)
        ball_y = float(p.y)

        standoff_m = float(self.get_parameter("standoff_m").value)
        min_ball_x_m = float(self.get_parameter("min_ball_x_m").value)
        max_goal_x_m = float(self.get_parameter("max_goal_x_m").value)
        max_goal_y_m = float(self.get_parameter("max_goal_y_m").value)
        goal_frame = str(self.get_parameter("goal_frame").value)

        if not math.isfinite(ball_x) or not math.isfinite(ball_y):
            return

        if ball_x < min_ball_x_m:
            if force_auto_mode:
                self._publish_mode("auto")
            self._lost_announced = False
            return

        goal_x = max(0.0, ball_x - standoff_m)
        goal_y = ball_y

        goal_x = min(goal_x, max_goal_x_m)
        goal_y = max(-max_goal_y_m, min(max_goal_y_m, goal_y))

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = goal_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0

        # Keep quaternion all-zeros so go_to_goal uses current yaw (no final heading alignment).
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

        self._goal_pub.publish(goal)
        if force_auto_mode:
            self._publish_mode("auto")

        self._lost_announced = False

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
