#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

class TestStanceController : public rclcpp::Node
{
public:
    TestStanceController() : Node("test_stance_controller")
    {
        // Publisher oluştur
        joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_position_controller/commands", 10);
        
        // Timer - 2 saniye bekle sonra komutu gönder
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TestStanceController::sendStanceCommand, this));
        
        RCLCPP_INFO(this->get_logger(), "Test Stance Controller başladı! 2 saniye sonra komut göndereceğim...");
    }

private:
    void sendStanceCommand()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        
        // 12 joint: fl, fr, bl, br (her biri hip, shoulder, knee)
        // Ayakta durma pozisyonu:
        msg.data = {
            0.0, 0.0, -1.57,   // fl: hip=0, shoulder=0, knee=-90deg (aşağı)
            0.0, 0.0, -1.57,   // fr
            0.0, 0.0, -1.57,   // bl
            0.0, 0.0, -1.57    // br
        };
        
        joint_cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "✅ Stance pozisyonu gönderildi! Robot ayakta durmalı.");
        
        // Bir kere gönder, timer'ı durdur
        timer_->cancel();
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestStanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}