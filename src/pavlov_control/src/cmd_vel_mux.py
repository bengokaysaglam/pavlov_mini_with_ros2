#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("teleop_topic", "/cmd_vel_teleop")
        self.declare_parameter("auto_topic", "/cmd_vel_auto")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/control_mode")
        self.declare_parameter("initial_mode", "teleop")
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.teleop_topic: str = self.get_parameter(
            "teleop_topic"
        ).get_parameter_value().string_value
        self.auto_topic: str = self.get_parameter(
            "auto_topic"
        ).get_parameter_value().string_value
        self.output_topic: str = self.get_parameter(
            "output_topic"
        ).get_parameter_value().string_value
        self.mode_topic: str = self.get_parameter(
            "mode_topic"
        ).get_parameter_value().string_value
        self.mode: str = self.get_parameter(
            "initial_mode"
        ).get_parameter_value().string_value.lower()
        self.cmd_timeout_sec: float = self.get_parameter(
            "cmd_timeout_sec"
        ).get_parameter_value().double_value
        publish_rate: float = self.get_parameter(
            "publish_rate_hz"
        ).get_parameter_value().double_value

        if self.mode not in ("teleop", "auto"):
            self.get_logger().warn(f"Invalid initial_mode='{self.mode}', switching to 'teleop'")
            self.mode = "teleop"

        self.teleop_sub = self.create_subscription(
            Twist, self.teleop_topic, self.teleop_callback, 10
        )
        self.auto_sub = self.create_subscription(
            Twist, self.auto_topic, self.auto_callback, 10
        )
        self.mode_sub = self.create_subscription(
            String, self.mode_topic, self.mode_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)

        self.last_teleop_cmd = Twist()
        self.last_auto_cmd = Twist()
        self.last_teleop_time = None
        self.last_auto_time = None

        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        timer_period = 1.0 / max(publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_selected_cmd, clock=self.wall_clock)

        self.get_logger().info(
            f"CMD_VEL_MUX READY | mode={self.mode} teleop={self.teleop_topic} "
            f"auto={self.auto_topic} out={self.output_topic}"
        )

    def teleop_callback(self, msg: Twist):
        self.last_teleop_cmd = msg
        self.last_teleop_time = self.wall_clock.now()

    def auto_callback(self, msg: Twist):
        self.last_auto_cmd = msg
        self.last_auto_time = self.wall_clock.now()

    def mode_callback(self, msg: String):
        requested = msg.data.strip().lower()
        if requested not in ("teleop", "auto"):
            self.get_logger().warn(
                f"Ignoring invalid mode='{msg.data}'. Use 'teleop' or 'auto'."
            )
            return
        if requested != self.mode:
            self.mode = requested
            self.get_logger().info(f"Control mode switched to '{self.mode}'")

    def _is_fresh(self, stamp):
        if stamp is None:
            return False
        age = (self.wall_clock.now() - stamp).nanoseconds * 1e-9
        return age <= self.cmd_timeout_sec

    def publish_selected_cmd(self):
        cmd = Twist()
        if self.mode == "teleop":
            if self._is_fresh(self.last_teleop_time):
                cmd = self.last_teleop_cmd
        else:
            if self._is_fresh(self.last_auto_time):
                cmd = self.last_auto_cmd

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
