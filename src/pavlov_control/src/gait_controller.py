#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class AdvancedBalancedGait(Node):
    def __init__(self):
        super().__init__("advanced_balanced_gait")

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.L1 = 0.03559
        self.L2 = 0.10000
        self.L3 = 0.10000

        self.step_height = 0.020
        self.stance_height = -0.15
        self.step_length = 0.05
        
        self.cog_shift_gain = 0.6      # CoG kayma katsayısı (0-1)
        self.trunk_pitch_gain = 0.02   # Pitch açısı katsayısı (rad)
        self.trunk_roll_gain = 0.02    # Roll açısı katsayısı (rad)
        
        self.hip_positions = [
            ( 0.09170,  0.05355),  # FL
            ( 0.09170, -0.05355),  # FR
            (-0.09170,  0.05355),  # BL
            (-0.09170, -0.05355),  # BR
        ]
        
        self.duty_cycle = 0.6
        self.total_phases = 40
        self.phase_increment = 1.0
        self.gait_phase = 0.0

        self.joint_names = [
            "hip1_fl", "hip2_fl", "knee_fl",
            "hip1_fr", "hip2_fr", "knee_fr",
            "hip1_bl", "hip2_bl", "knee_bl",
            "hip1_br", "hip2_br", "knee_br"
        ]
        
        self.started = False
        self.timer_start = self.create_timer(3.0, self.start_gait)
        
        self.get_logger().info("    ADVANCED BALANCED GAIT CONTROLLER")                 

    def InverseKinematics(self, x, y, z):
        try:
            # Robotu hedefe dogru dondurur.
            theta1 = math.atan2(y, -z)

            # Robota tabanindan baktigimizde Y-Z duzleminde gorulen kus ucusu mesafesidir.
            r_yz = math.sqrt(y**2 + z**2)

            d = r_yz - self.L1
            # hedefe ulasmak icin kolun ne kadar uzanmasi gerektigini hesaplariz.
            D = math.sqrt(x**2 + d**2)

            # hedefe ulasmak icin dirsek kisminin ne kadar bukulecegini hesaplariz.
            cos_theta3 = (self.L2**2 + self.L3**2 - D**2) / (2 * self.L2 * self.L3)
            cos_theta3 = max(-1, min(1, cos_theta3))                                          
            theta3 = math.acos(cos_theta3) - math.pi

            # omuz ekleminin ne kadar kalkacagini hesaplariz.
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
            self.get_logger().error(f"❌ IK Error: {e}")
            return 0.0, 0.0, 0.0

    def smooth_interpolate(self, t):
        return t * t * t * (t * (t * 6 - 15) + 10)

    def get_swing_trajectory(self, t):
        t_smooth = self.smooth_interpolate(t)
        x = -self.step_length/2 + self.step_length * t_smooth
        
        if t < 0.5:
            z_progress = self.smooth_interpolate(t * 2)
            z = self.step_height * z_progress
        else:
            z_progress = self.smooth_interpolate((1.0 - t) * 2)
            z = self.step_height * z_progress
            
        return x, z

    def get_stance_trajectory(self, t):
        t_smooth = self.smooth_interpolate(t)
        x = self.step_length/2 - self.step_length * t_smooth
        z = 0.0
        return x, z

    def apply_trunk_orientation(self, x, y, z, leg_idx, pitch, roll):
        """
        Gövde eğimini bacak pozisyonuna uygular.
        
        Parametreler:
            x, y, z: Bacak pozisyonu
            leg_idx: Bacak indeksi
            pitch: Yunuslama açısı (rad, + ön yukarı)
            roll: Yalpalama açısı (rad, + sağ yukarı)
        
        Dönüş:
            (x', y', z'): Eğim uygulanmış pozisyonlar
        """
        hip_x, hip_y = self.hip_positions[leg_idx]
        
        # Pitch etkisi (ön-arka eğim)
        # Ön bacaklar pitch ile yukarı/aşağı gider
        z_pitch = hip_x * math.sin(pitch)
        
        # Roll etkisi (sağ-sol eğim)
        # Sağ bacaklar roll ile yukarı/aşağı gider
        # Negatif çünkü +roll = sağ yukarı
        z_roll = -hip_y * math.sin(roll) 
        
        # Toplam z offseti
        z_adjusted = z + z_pitch + z_roll
        
        return x, y, z_adjusted

    def calculate_trunk_compensation(self, phase):
        """
        Gövde kompanzasyonunu hesapla
        
        Dönüş:
            trunk_x, trunk_y: Gövde X-Y kayması
            trunk_pitch: Gövde pitch açısı
            trunk_roll: Gövde roll açısı
            stance_legs: Yerdeki bacaklar
        """
        # Hangi bacaklar stance'de?
        stance_legs = []
        
        for leg_idx in range(4):
            if leg_idx in [0, 3]:
                leg_phase = (phase % self.total_phases) / self.total_phases
            else:
                leg_phase = ((phase + self.total_phases/2) % self.total_phases) / self.total_phases
            
            swing_duration = 1.0 - self.duty_cycle
            
            if leg_phase >= swing_duration:
                stance_legs.append(leg_idx)
        
        # Destek merkezi
        if len(stance_legs) >= 2:
            support_x = sum(self.hip_positions[i][0] for i in stance_legs) / len(stance_legs)
            support_y = sum(self.hip_positions[i][1] for i in stance_legs) / len(stance_legs)
            
            # Gövde kayması (CoG kompanzasyonu)
            trunk_x = support_x * self.cog_shift_gain
            trunk_y = support_y * self.cog_shift_gain
            
            # Eğer destek ön tarafta -> gövdeyi one egdiririz.
            trunk_pitch = -support_x * self.trunk_pitch_gain
            
            # Eğer destek sağda -> gövde sağa egdiririz.
            trunk_roll = -support_y * self.trunk_roll_gain
        else:
            trunk_x = 0.0
            trunk_y = 0.0
            trunk_pitch = 0.0
            trunk_roll = 0.0
        
        return trunk_x, trunk_y, trunk_pitch, trunk_roll, stance_legs

    def get_leg_position(self, leg_index, phase):
        if leg_index in [0, 3]:
            leg_phase = (phase % self.total_phases) / self.total_phases
        else:
            leg_phase = ((phase + self.total_phases/2) % self.total_phases) / self.total_phases

        swing_duration = 1.0 - self.duty_cycle
        
        if leg_phase < swing_duration:
            t = leg_phase / swing_duration
            x_offset, z_offset = self.get_swing_trajectory(t)
            is_stance = False
        else:
            t = (leg_phase - swing_duration) / self.duty_cycle
            x_offset, z_offset = self.get_stance_trajectory(t)
            is_stance = True
        
        x = x_offset
        y = self.L1
        z = self.stance_height + z_offset
        
        return x, y, z, is_stance

    def start_gait(self):
        if self.started:
            return
            
        self.get_logger().info(" ADVANCED BALANCED GAIT BAŞLIYOR!")
        self.started = True
        self.timer_start.cancel()
        
        self.timer_gait = self.create_timer(0.05, self.gait_loop)

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
                
                # CoG kompanzasyonu uygular
                if is_stance:
                    x_comp = x - trunk_x
                    y_comp = y - trunk_y
                else:
                    x_comp = x - trunk_x * 0.5
                    y_comp = y - trunk_y * 0.5
                
                # Gövde eğimi uygular
                x_final, y_final, z_final = self.apply_trunk_orientation(
                    x_comp, y_comp, z, leg_idx, trunk_pitch, trunk_roll
                )
                
                if leg_idx in [1, 3]:
                    y_ik = -y_final
                else:
                    y_ik = y_final
                
                theta1, theta2, theta3 = self.InverseKinematics(x_final, y_ik, z_final)
                
                positions.extend([theta1, theta2, theta3])
                velocities.extend([0.0, 0.0, 0.0])
            
            point.positions = positions
            point.velocities = velocities
            point.accelerations = []
            point.effort = []
            point.time_from_start = Duration(sec=0, nanosec=40_000_000)
            
            msg.points = [point]
            self.publisher_.publish(msg)
            
            if int(self.gait_phase) % 10 == 0:
                leg_names = ["FL", "FR", "BL", "BR"]
                stance_names = [leg_names[i] for i in stance_legs]
                
                self.get_logger().info(
                    f"⚖️  Faz {int(self.gait_phase):2d} | "
                    f"Stance: {'+'.join(stance_names):8s} | "
                    f"X={trunk_x:+.3f} Y={trunk_y:+.3f} | "
                    f"Pitch={math.degrees(trunk_pitch):+.1f}° Roll={math.degrees(trunk_roll):+.1f}°"
                )
            
            self.gait_phase = (self.gait_phase + self.phase_increment) % self.total_phases
            
        except Exception as e:
            self.get_logger().error(f"❌ Gait Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedBalancedGait()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Yürüyüş durduruldu.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
