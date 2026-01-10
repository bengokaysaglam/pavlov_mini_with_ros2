#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <cmath>

class SimpleStepController : public rclcpp::Node
{
public:
    SimpleStepController() : Node("simple_step_controller"), step_phase_(0), time_(0.0)
    {
        // Joint Trajectory Publisher (Gazebo bu topic'i dinler)
        joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/set_joint_trajectory", 10);
        
        // Timer - 20ms (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimpleStepController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "🚶 Basit Adım Atma Başladı! (Gazebo Direkt Mod)");
        RCLCPP_INFO(this->get_logger(), "Not: Eğer robot hareket etmezse, topic'leri kontrol edin:");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic list | grep joint");
    }

private:
    void controlLoop()
    {
        time_ += 0.1; // 100ms increment
        
        auto msg = trajectory_msgs::msg::JointTrajectory();
        
        // Joint isimleri
        msg.joint_names = {
            "fl_hip_joint", "fl_shoulder_joint", "fl_knee_joint",
            "fr_hip_joint", "fr_shoulder_joint", "fr_knee_joint",
            "bl_hip_joint", "bl_shoulder_joint", "bl_knee_joint",
            "br_hip_joint", "br_shoulder_joint", "br_knee_joint"
        };
        
        // Trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        
        // Varsayılan stance pozisyonu
        double default_hip = 0.0;
        double default_shoulder = 0.2;
        double default_knee = -1.4;
        
        for (int i = 0; i < 4; i++) {
            point.positions[i*3 + 0] = default_hip;
            point.positions[i*3 + 1] = default_shoulder;
            point.positions[i*3 + 2] = default_knee;
        }
        
        // FAZ GEÇİŞLERİ
        if (time_ < 2.0) {
            step_phase_ = 0;
        }
        else if (time_ < 3.0) {
            if (step_phase_ != 1) {
                step_phase_ = 1;
                RCLCPP_INFO(this->get_logger(), "✅ Faz 1: FL bacağı kaldırılıyor...");
            }
            point.positions[0] = 0.0;
            point.positions[1] = 0.6;
            point.positions[2] = -0.8;
        }
        else if (time_ < 4.0) {
            if (step_phase_ != 2) {
                step_phase_ = 2;
                RCLCPP_INFO(this->get_logger(), "✅ Faz 2: FL bacağı öne taşınıyor...");
            }
            point.positions[0] = 0.3;
            point.positions[1] = 0.6;
            point.positions[2] = -0.8;
        }
        else if (time_ < 5.0) {
            if (step_phase_ != 3) {
                step_phase_ = 3;
                RCLCPP_INFO(this->get_logger(), "✅ Faz 3: FL bacağı indiriliyor...");
            }
            point.positions[0] = 0.3;
            point.positions[1] = 0.2;
            point.positions[2] = -1.4;
        }
        else if (time_ < 7.0) {
            if (step_phase_ != 4) {
                step_phase_ = 4;
                RCLCPP_INFO(this->get_logger(), "✅ Faz 4: TROT - FL ve BR hareket...");
            }
            
            double t = time_ - 5.0;
            double swing_progress = t / 2.0;
            double lift_height = 0.4 * std::sin(M_PI * swing_progress);
            
            // FL
            point.positions[0] = 0.3 * std::cos(M_PI * swing_progress);
            point.positions[1] = 0.2 + lift_height;
            point.positions[2] = -1.4 + 0.6 * std::sin(M_PI * swing_progress);
            
            // BR
            point.positions[9] = 0.3 * std::cos(M_PI * swing_progress);
            point.positions[10] = 0.2 + lift_height;
            point.positions[11] = -1.4 + 0.6 * std::sin(M_PI * swing_progress);
        }
        else {
            if (step_phase_ != 5) {
                step_phase_ = 5;
                RCLCPP_INFO(this->get_logger(), "✅ Tamamlandı!");
            }
        }
        
        msg.points.push_back(point);
        joint_traj_pub_->publish(msg);
        
        // Debug
        if (time_ < 0.5) {
            RCLCPP_INFO(this->get_logger(), "📤 Komut gönderiliyor: /set_joint_trajectory");
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_phase_;
    double time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleStepController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}