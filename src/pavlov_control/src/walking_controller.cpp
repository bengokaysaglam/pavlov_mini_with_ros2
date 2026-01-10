#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "kinematics.hpp"
#include "gait_scheduler.hpp"
#include "force_controller.hpp"

using namespace pavlov_control;

class WalkingController : public rclcpp::Node
{
public:
    WalkingController() : Node("walking_controller")
    {
        // Kinematics nesnelerini oluştur (her bacak için aynı boyutlar)
        double hip_offset = 0.05;
        double thigh = 0.12;
        double shank = 0.12;
        
        for (int i = 0; i < 4; i++) {
            leg_kin_[i] = std::make_unique<LegKinematics>(hip_offset, thigh, shank);
        }
        
        // Gait scheduler
        gait_ = std::make_unique<GaitScheduler>();
        gait_->setGaitType(GaitType::TROT);
        gait_->setStepParameters(0.05, 0.10);  // 5cm yükseklik, 10cm uzunluk
        gait_->setFrequency(1.0);  // 1Hz
        
        // Force controller
        force_ctrl_ = std::make_unique<ForceController>();
        
        // Nominal stance pozisyonları (body frame)
        nominal_foot_pos_[0] = Eigen::Vector3d(0.15, 0.10, -0.20);   // FL
        nominal_foot_pos_[1] = Eigen::Vector3d(0.15, -0.10, -0.20);  // FR
        nominal_foot_pos_[2] = Eigen::Vector3d(-0.15, 0.10, -0.20);  // BL
        nominal_foot_pos_[3] = Eigen::Vector3d(-0.15, -0.10, -0.20); // BR
        
        // ROS publishers/subscribers
        joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_position_controller/commands", 10);
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WalkingController::cmdVelCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&WalkingController::imuCallback, this, std::placeholders::_1));
        
        // Control loop - 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&WalkingController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "🚶 Walking Controller başladı!");
        RCLCPP_INFO(this->get_logger(), "Komut vermek için: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...");
    }

private:
    void controlLoop()
    {
        double dt = 0.02;  // 50Hz
        
        // Gait scheduler'ı güncelle
        gait_->update(dt);
        
        // Hız komutunu normalize et (maksimum 1.0)
        Eigen::Vector3d vel_cmd(
            std::max(-1.0, std::min(1.0, desired_velocity_.linear.x)),
            std::max(-1.0, std::min(1.0, desired_velocity_.linear.y)),
            0.0
        );
        
        // Her bacak için trajectory hesapla
        std::array<Eigen::Vector3d, 4> target_foot_pos;
        std::array<bool, 4> stance_legs;
        
        for (int i = 0; i < 4; i++) {
            target_foot_pos[i] = gait_->getFootTrajectory(i, nominal_foot_pos_[i], vel_cmd);
            stance_legs[i] = gait_->isLegInStance(i);
        }
        
        // Inverse kinematics ile joint açılarını hesapla
        std::array<Eigen::Vector3d, 4> joint_angles;
        for (int i = 0; i < 4; i++) {
            joint_angles[i] = leg_kin_[i]->inverseKinematics(target_foot_pos[i]);
        }
        
        // Joint komutlarını gönder
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(12);
        
        // Sıra: FL, FR, BL, BR (her biri hip, shoulder, knee)
        for (int i = 0; i < 4; i++) {
            msg.data[i*3 + 0] = joint_angles[i][0];  // hip
            msg.data[i*3 + 1] = joint_angles[i][1];  // shoulder
            msg.data[i*3 + 2] = joint_angles[i][2];  // knee
        }
        
        joint_cmd_pub_->publish(msg);
        
        // Debug - her 2 saniyede bir
        if (++debug_counter_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Gait phase: %.2f | Vel: (%.2f, %.2f) | Stance: [%d %d %d %d]",
                gait_->getPhase(),
                vel_cmd.x(), vel_cmd.y(),
                stance_legs[0], stance_legs[1], stance_legs[2], stance_legs[3]);
        }
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        desired_velocity_ = *msg;
        
        // Hıza göre gait frekansını ayarla
        double speed = std::sqrt(msg->linear.x * msg->linear.x + 
                                msg->linear.y * msg->linear.y);
        gait_->setFrequency(0.5 + speed * 1.5);  // 0.5 - 2.0 Hz
        
        RCLCPP_INFO(this->get_logger(), "Yeni hız komutu: x=%.2f, y=%.2f", 
                    msg->linear.x, msg->linear.y);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMU verilerini sakla (şu an kullanmıyoruz ama force control için lazım)
        current_orientation_ = msg->orientation;
    }

    // Kinematics nesneleri (4 bacak)
    std::array<std::unique_ptr<LegKinematics>, 4> leg_kin_;
    
    // Gait scheduler
    std::unique_ptr<GaitScheduler> gait_;
    
    // Force controller
    std::unique_ptr<ForceController> force_ctrl_;
    
    // Nominal foot positions
    std::array<Eigen::Vector3d, 4> nominal_foot_pos_;
    
    // ROS
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    geometry_msgs::msg::Twist desired_velocity_;
    geometry_msgs::msg::Quaternion current_orientation_;
    int debug_counter_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WalkingController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}