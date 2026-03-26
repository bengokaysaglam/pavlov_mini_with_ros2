#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class GaitController(Node):
    def __init__(self):
        super().__init__("gait_controller")

        self.declare_parameter("max_step_length", 0.10)
        self.declare_parameter("gait_timer_period", 0.05)
        self.declare_parameter("phase_increment", 2.0)
        self.declare_parameter("cmd_alpha_linear", 0.25)
        self.declare_parameter("cmd_alpha_angular", 0.20)
        
        self.publisher_ = self.create_publisher(JointTrajectory,"/joint_trajectory_controller/joint_trajectory",10)
        self.cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self.cmd_vel_callback,10)

        # ROBOT LEG DIMENSIONS
        self.L1 = 0.03559
        self.L2 = 0.10000
        self.L3 = 0.10000

        # GAIT PARAMETERS
        self.step_height = 0.020
        self.stance_height = -0.15
        self.base_step_length = 0.00
        self.max_step_length = float(self.get_parameter("max_step_length").value)
        self.leg_step_alpha = 0.25
        
        # TURNING PARAMETERS
        self.wheelbase = 0.1834
        self.track_width = 0.1071
        self.limit_turn_to_avoid_counter_step = True
        self.turn_limit_min_linear = 0.01
        self.turn_reduction_gain = 0.5
        
        # TRUNK COMPENSATION GAINS
        self.cog_shift_gain = 0.6
        self.trunk_pitch_gain = 0.02
        self.trunk_roll_gain = 0.02
        
        # HIP POSITIONS
        self.hip_positions = [
            ( 0.09170,  0.05355),  # FL
            ( 0.09170, -0.05355),  # FR
            (-0.09170,  0.05355),  # BL
            (-0.09170, -0.05355),  # BR
        ]
        
        # GAIT TIMING
        self.duty_cycle = 0.6
        self.total_phases = 40
        self.phase_increment = float(self.get_parameter("phase_increment").value)
        self.gait_timer_period = float(self.get_parameter("gait_timer_period").value)
        self.gait_phase = 0.0

        # JOINT NAMES
        self.joint_names = [
            "hip1_fl", "hip2_fl", "knee_fl",
            "hip1_fr", "hip2_fr", "knee_fr",
            "hip1_bl", "hip2_bl", "knee_bl",
            "hip1_br", "hip2_br", "knee_br"
        ]
        
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.filtered_linear_x = 0.0
        self.filtered_angular_z = 0.0
        self.cmd_alpha_linear = float(self.get_parameter("cmd_alpha_linear").value)
        self.cmd_alpha_angular = float(self.get_parameter("cmd_alpha_angular").value)
        
        # PER-LEG STEP LENGTHS
        self.leg_step_lengths = [0.0, 0.0, 0.0, 0.0]
        
        self.is_moving = False
        self.movement_start_threshold = 0.006
        self.movement_stop_threshold = 0.003
        
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.started = False
        self.timer_start = self.create_timer(3.0, self.start_gait, clock=self.wall_clock)
        
        self.get_logger().info("GAIT CONTROLLER INITIALIZED")

    def cmd_vel_callback(self, msg):

        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z

        # Smooth command inputs to avoid gait jitter from fast cmd_vel fluctuations.
        self.filtered_linear_x += self.cmd_alpha_linear * (self.linear_x - self.filtered_linear_x)
        self.filtered_angular_z += self.cmd_alpha_angular * (self.angular_z - self.filtered_angular_z)
        
        dt = self.gait_timer_period * self.total_phases / max(self.phase_increment, 1e-6)
        self.base_step_length = self.filtered_linear_x * dt
        
        effective_angular_z = self.filtered_angular_z

        if self.limit_turn_to_avoid_counter_step and abs(self.filtered_linear_x) > self.turn_limit_min_linear:

            # v_left = v - (w * track_width / 2)
            # v_right = v + (w * track_width / 2)
            # To avoid counter-stepping, we need to ensure that neither v_left nor v_right goes negative when moving forward, or positive when moving backward. This gives us:
            # For forward motion: w <= (2 * v) / track_width
            # For backward motion: w >= (2 * v) / track_width
            max_w_no_counter_step = (2.0 * abs(self.filtered_linear_x)) / max(self.track_width, 1e-6)

            # If the commanded turn rate exceeds the limit, we clamp it and log a warning.
            clamped_w = max(-max_w_no_counter_step, min(max_w_no_counter_step, self.filtered_angular_z))

            if abs(clamped_w - self.filtered_angular_z) > 1e-6:
                self.get_logger().info(f"Turn clamped: wz {self.filtered_angular_z:.2f} -> {clamped_w:.2f}",throttle_duration_sec=1.0)
            # Use the clamped turn rate for gait calculations to prevent counter-stepping.
            effective_angular_z = clamped_w
        
        # Calculate turn differential for step length adjustment. Positive turn_differential means the left legs take shorter steps and the right legs take longer steps, causing a turn to the left.
        # The term (turn_reduction_gain) allows us to reduce the turn effect to prevent overcompensation, which can cause jitter or instability at low speeds. 
        turn_differential = (effective_angular_z * self.turn_reduction_gain * self.track_width * dt / 2.0)

        left_step_target = self.base_step_length - turn_differential
        right_step_target = self.base_step_length + turn_differential

        # If the robot is commanded to move forward, we want to prevent the inner legs from taking negative steps 
        # which would cause them to lift and step backwards (counter-stepping). Similarly, if moving backward, 
        # we want to prevent them from taking positive steps. This logic ensures that when moving forward, both 
        # legs take positive steps, and when moving backward, both legs take negative steps, thus maintaining 
        # stability and preventing counter-stepping.
        if self.filtered_linear_x > self.turn_limit_min_linear:
            left_step_target = max(0.0, left_step_target)
            right_step_target = max(0.0, right_step_target)

        elif self.filtered_linear_x < -self.turn_limit_min_linear:
            left_step_target = min(0.0, left_step_target)
            right_step_target = min(0.0, right_step_target)

        # Finally, we clamp the step targets to the maximum allowed step length to ensure we don't 
        # command unrealistic steps that could destabilize the robot.
        left_step_target = max(-self.max_step_length, min(self.max_step_length, left_step_target))
        right_step_target = max(-self.max_step_length, min(self.max_step_length, right_step_target))

        target_steps = [
            left_step_target,
            right_step_target,
            left_step_target,
            right_step_target
        ]
        
        for idx in range(4):
            self.leg_step_lengths[idx] += self.leg_step_alpha * (target_steps[idx] - self.leg_step_lengths[idx])

        left_step = self.leg_step_lengths[0]
        right_step = self.leg_step_lengths[1]
        
        max_step = max(abs(s) for s in self.leg_step_lengths)
        
        if max_step > self.movement_start_threshold:
            if not self.is_moving:
                self.get_logger().info("Starting movement")
                self.is_moving = True
        else:
            if self.is_moving and max_step < self.movement_stop_threshold:
                self.get_logger().info("Stopping movement")
                self.is_moving = False
        
        if abs(self.filtered_linear_x) > 0.01 or abs(self.filtered_angular_z) > 0.01:
            self.get_logger().info(f"CMD: vx={self.filtered_linear_x:.2f} wz={effective_angular_z:.2f} → "f"L={left_step:.3f}m R={right_step:.3f}m",throttle_duration_sec=1.0)

    def inverse_kinematics(self, x, y, z):
        try:
            theta1 = math.atan2(y, -z)
            r_yz = math.sqrt(y**2 + z**2)
            d = r_yz - self.L1
            D = math.sqrt(x**2 + d**2)
            
            cos_theta3 = (self.L2**2 + self.L3**2 - D**2) / (2 * self.L2 * self.L3)
            cos_theta3 = max(-1, min(1, cos_theta3))
            theta3 = math.acos(cos_theta3) - math.pi
            
            alpha = math.atan2(-x, d)
            beta_denom = 2 * self.L2 * D
            
            if abs(beta_denom) < 1e-6:
                beta = 0
            else:
                cos_beta = (self.L2**2 + D**2 - self.L3**2) / beta_denom
                cos_beta = max(-1.0, min(1.0, cos_beta))
                beta = math.acos(cos_beta)
                
            theta2 = alpha + beta
            return theta1, theta2, theta3
            
        except Exception as e:
            self.get_logger().error(f"IK Error: {e}")
            return 0.0, 0.0, 0.0

    def smooth_interpolate(self, t):
        return t * t * t * (t * (t * 6 - 15) + 10)

    # This function generates the foot trajectory during the swing phase. It uses a smooth interpolation 
    # to create a natural stepping motion, where the foot lifts up to the specified step height at the 
    # midpoint of the swing and then comes back down to the stance height by the end of the swing.
    def get_swing_trajectory(self, t, step_length):
        t_smooth = self.smooth_interpolate(t)
        x = -step_length / 2 + step_length * t_smooth
        
        if t < 0.5:
            z_progress = self.smooth_interpolate(t * 2)
            z = self.step_height * z_progress
        else:
            z_progress = self.smooth_interpolate((1.0 - t) * 2)
            z = self.step_height * z_progress
            
        return x, z

    def get_stance_trajectory(self, t, step_length):
        t_smooth = self.smooth_interpolate(t)
        x = step_length / 2 - step_length * t_smooth
        z = 0.0
        return x, z

    def apply_trunk_orientation(self, x, y, z, leg_idx, pitch, roll):
        hip_x, hip_y = self.hip_positions[leg_idx]
        z_pitch = hip_x * math.sin(pitch)
        z_roll = -hip_y * math.sin(roll)
        z_adjusted = z + z_pitch + z_roll
        return x, y, z_adjusted

    def calculate_trunk_compensation(self, phase):
        stance_legs = []
        
        for leg_idx in range(4):
            # Determine leg phase based on gait phase and leg index. Legs 0 and 3 (FL and BR) are in phase 
            # while legs 1 and 2 (FR and BL) are out of phase by half a cycle. This creates the alternating tripod gait pattern.
            if leg_idx in [0, 3]:
                leg_phase = (phase % self.total_phases) / self.total_phases
            # For legs 1 and 2, we add half the total phases to create the out-of-phase relationship.
            # The modulo operation ensures that the phase wraps around correctly.
            else:
                leg_phase = ((phase + self.total_phases/2) % self.total_phases) / self.total_phases
            
            swing_duration = 1.0 - self.duty_cycle
            
            if leg_phase >= swing_duration:
                stance_legs.append(leg_idx)
        
        if len(stance_legs) >= 2:
            # Calculate the average position of the stance legs to determine the support polygon center.
            # This is a simple way to estimate where the robot's weight is supported, and we can use this 
            # information to shift the trunk in the opposite direction to help maintain balance.
            support_x = sum(self.hip_positions[i][0] for i in stance_legs) / len(stance_legs)
            support_y = sum(self.hip_positions[i][1] for i in stance_legs) / len(stance_legs)
            
            trunk_x = support_x * self.cog_shift_gain
            trunk_y = support_y * self.cog_shift_gain
            trunk_pitch = -support_x * self.trunk_pitch_gain
            trunk_roll = -support_y * self.trunk_roll_gain
        else:
            trunk_x = 0.0
            trunk_y = 0.0
            trunk_pitch = 0.0
            trunk_roll = 0.0
        
        return trunk_x, trunk_y, trunk_pitch, trunk_roll, stance_legs

    def get_leg_position(self, leg_index, phase):
        step_length = self.leg_step_lengths[leg_index]
        
        if not self.is_moving:
            x = 0.0
            y = self.L1
            z = self.stance_height
            return x, y, z, True
        
        if leg_index in [0, 3]:
            leg_phase = (phase % self.total_phases) / self.total_phases
        else:
            leg_phase = ((phase + self.total_phases/2) % self.total_phases) / self.total_phases

        swing_duration = 1.0 - self.duty_cycle
        
        if leg_phase < swing_duration:
            t = leg_phase / swing_duration
            x_offset, z_offset = self.get_swing_trajectory(t, step_length)
            is_stance = False
        else:
            t = (leg_phase - swing_duration) / self.duty_cycle
            x_offset, z_offset = self.get_stance_trajectory(t, step_length)
            is_stance = True
        
        x = x_offset
        y = self.L1
        z = self.stance_height + z_offset
        
        return x, y, z, is_stance

    def start_gait(self):
        if self.started:
            return
            
        self.get_logger().info("GAIT CONTROLLER STARTED!")
        self.started = True
        self.timer_start.cancel()
        
        self.timer_gait = self.create_timer(
            self.gait_timer_period,
            self.gait_loop,
            clock=self.wall_clock
        )

    def gait_loop(self):
        try:
            trunk_x, trunk_y, trunk_pitch, trunk_roll, stance_legs = \
                self.calculate_trunk_compensation(self.gait_phase)
            
            msg = JointTrajectory()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = ''
            msg.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            positions = []
            velocities = []
            
            for leg_idx in range(4):
                x, y, z, is_stance = self.get_leg_position(leg_idx, self.gait_phase)
                
                # Apply CoG compensation
                if is_stance:
                    x_comp = x - trunk_x
                    y_comp = y - trunk_y
                else:
                    x_comp = x - trunk_x * 0.5
                    y_comp = y - trunk_y * 0.5
                
                # Apply trunk orientation
                x_final, y_final, z_final = self.apply_trunk_orientation(
                    x_comp, y_comp, z, leg_idx, trunk_pitch, trunk_roll
                )
                
                # Mirror Y for right side
                if leg_idx in [1, 3]: 
                    y_ik = -y_final
                else:
                    y_ik = y_final
                
                theta1, theta2, theta3 = self.inverse_kinematics(x_final, y_ik, z_final)
                
                positions.extend([theta1, theta2, theta3])
                velocities.extend([0.0, 0.0, 0.0])
            
            point.positions = positions
            point.velocities = velocities
            point.accelerations = []
            point.effort = []
            point.time_from_start = Duration(sec=0, nanosec=40_000_000)
            
            msg.points = [point]
            self.publisher_.publish(msg)
            
            if self.is_moving:
                self.gait_phase = (self.gait_phase + self.phase_increment) % self.total_phases
            
            if self.is_moving and int(self.gait_phase) % 10 == 0:
                leg_names = ["FL", "FR", "BL", "BR"]
                stance_names = [leg_names[i] for i in stance_legs]
                
                self.get_logger().info(
                    f"Phase {int(self.gait_phase):2d} | "
                    f"Stance: {'+'.join(stance_names):8s} | "
                    f"Steps: L={self.leg_step_lengths[0]:.3f} R={self.leg_step_lengths[1]:.3f}",
                    throttle_duration_sec=2.0
                )
            
        except Exception as e:
            self.get_logger().error(f"Gait Loop Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GaitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n Gait controller stopped.")
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()