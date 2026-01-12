#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class PavlovGaitController(Node):
    def __init__(self):
        super().__init__("pavlov_gait_controller")

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # Robot parametreleri
        self.L1 = 0.03559
        self.L2 = 0.10000
        self.L3 = 0.10000

        # Yürüyüş parametreleri - İYİLEŞTİRİLMİŞ
        self.step_height = 0.025        # Biraz daha yüksek adım
        self.stance_height = -0.15      # SABİT yükseklik (eğilme yok!)
        self.step_length = 0.05         # Biraz daha uzun adım
        
        # Gait timing
        self.duty_cycle = 0.6           # %60 stance, %40 swing (daha stabil)
        self.total_phases = 40          # ÇOK DAHA FAZLA faz = ultra smooth
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
        
        self.get_logger().info("🐕 Ultra Smooth Gait Controller")
        self.get_logger().info(f"   Faz sayısı: {self.total_phases}")
        self.get_logger().info(f"   Adım uzunluğu: {self.step_length*100:.1f}cm")
        self.get_logger().info(f"   Yükseklik: {abs(self.stance_height)*100:.1f}cm")
        self.get_logger().info("3 saniye sonra başlıyor...")

    def InverseKinematics(self, x, y, z):
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
        """
        Quintic (5th order) polynomial - En smooth interpolasyon
        t: 0-1 arası
        Returns: smooth 0-1 değeri
        """
        return t * t * t * (t * (t * 6 - 15) + 10)

    def bezier_trajectory(self, t, start, mid, end):
        """
        Quadratic Bezier curve - Çok smooth yol
        """
        return (1-t)**2 * start + 2*(1-t)*t * mid + t**2 * end

    def get_swing_trajectory(self, t):
        """
        Swing fazı için smooth trajectory
        t: 0-1 arası normalize edilmiş zaman
        """
        # X hareketi - smooth başla, smooth bitir
        t_smooth = self.smooth_interpolate(t)
        x = -self.step_length/2 + self.step_length * t_smooth
        
        # Z hareketi - Bezier curve ile smooth parabolik lift
        # Bacak önce hızlı kalkar, sonra yavaş iner
        if t < 0.5:
            # Kalkış fazı - hızlı
            z_progress = self.smooth_interpolate(t * 2)
            z = self.step_height * z_progress
        else:
            # İniş fazı - yavaş ve smooth
            z_progress = self.smooth_interpolate((1.0 - t) * 2)
            z = self.step_height * z_progress
            
        return x, z

    def get_stance_trajectory(self, t):
        """
        Stance fazı - sabit hızda geri
        """
        t_smooth = self.smooth_interpolate(t)
        x = self.step_length/2 - self.step_length * t_smooth
        z = 0.0
        return x, z

    def get_leg_position(self, leg_index, phase):
        """
        TROT GAIT with improved timing
        """
        # Hangi bacak çifti
        if leg_index in [0, 3]:  # FL ve BR
            leg_phase = (phase % self.total_phases) / self.total_phases
        else:  # FR ve BL
            leg_phase = ((phase + self.total_phases/2) % self.total_phases) / self.total_phases

        # Duty cycle'a göre swing/stance ayır
        swing_duration = 1.0 - self.duty_cycle  # 0.4
        
        if leg_phase < swing_duration:
            # Swing phase
            t = leg_phase / swing_duration
            x_offset, z_offset = self.get_swing_trajectory(t)
        else:
            # Stance phase
            t = (leg_phase - swing_duration) / self.duty_cycle
            x_offset, z_offset = self.get_stance_trajectory(t)
        
        x = x_offset
        y = self.L1
        z = self.stance_height + z_offset  # SABİT yükseklik + sadece swing offset
        
        return x, y, z

    def start_gait(self):
        if self.started:
            return
            
        self.get_logger().info("🚀 ULTRA SMOOTH YÜRÜYÜŞ BAŞLIYOR!")
        self.started = True
        self.timer_start.cancel()
        
        # Çok sık güncelleme = ultra smooth
        self.timer_gait = self.create_timer(0.05, self.gait_loop)  # 20 Hz

    def gait_loop(self):
        try:
            msg = JointTrajectory()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = ''
            msg.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            
            positions = []
            velocities = []
            
            for leg_idx in range(4):
                x, y, z = self.get_leg_position(leg_idx, self.gait_phase)
                
                if leg_idx in [1, 3]:  # Sağ bacaklar (FR, BR)
                    theta1, theta2, theta3 = self.InverseKinematics(x, -y, z)
                else:  # Sol bacaklar (FL, BL)
                    theta1, theta2, theta3 = self.InverseKinematics(x, y, z)
                
                positions.extend([theta1, theta2, theta3])
                velocities.extend([0.0, 0.0, 0.0])  # Controller hesaplasın
            
            point.positions = positions
            point.velocities = velocities
            point.accelerations = []
            point.effort = []
            point.time_from_start = Duration(sec=0, nanosec=40_000_000)  # 0.04 saniye
            
            msg.points = [point]
            self.publisher_.publish(msg)
            
            # Faz ilerlet
            self.gait_phase = (self.gait_phase + self.phase_increment) % self.total_phases
            
            # Her saniyede bir log
            if int(self.gait_phase) % 20 == 0:
                cycle = int(self.gait_phase / self.total_phases)
                self.get_logger().info(f"Yürüyüş döngüsü: {cycle}, Faz: {self.gait_phase:.0f}")
            
        except Exception as e:
            self.get_logger().error(f"Gait Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PavlovGaitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()