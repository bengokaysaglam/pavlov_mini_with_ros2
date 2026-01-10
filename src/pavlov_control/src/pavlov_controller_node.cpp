// pavlov_controller_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <memory>
#include <array>

// Gait tipi
enum class GaitType {
    TROT,
    PACE,
    DIAGONAL,
    STAND
};

// Her bacak için faz bilgisi
struct LegState {
    Eigen::Vector3d position;      // Ayak pozisyonu (body frame)
    Eigen::Vector3d velocity;      // Ayak hızı
    Eigen::Vector3d force;         // Ayak kuvveti
    bool in_contact;               // Zeminde mi?
    double phase;                  // Gait fazı [0, 1]
};

// Robot durumu
struct RobotState {
    Eigen::Vector3d position;          // Gövde pozisyonu
    Eigen::Vector3d velocity;          // Gövde hızı
    Eigen::Quaterniond orientation;    // Gövde yönelimi
    Eigen::Vector3d angular_velocity;  // Açısal hız
    std::array<LegState, 4> legs;      // 4 bacak durumu
};

class GaitScheduler {
public:
    GaitScheduler() 
        : phase_(0.0), 
          frequency_(1.0), 
          duty_factor_(0.5),
          step_height_(0.05),
          step_length_(0.1)
    {
        setGaitType(GaitType::TROT);
    }

    void setGaitType(GaitType type) {
        current_gait_ = type;
        switch(type) {
            case GaitType::TROT:
                // Çapraz bacaklar aynı anda
                phase_offsets_ = {0.0, 0.5, 0.5, 0.0};
                break;
            case GaitType::PACE:
                // Yan bacaklar aynı anda
                phase_offsets_ = {0.0, 0.5, 0.0, 0.5};
                break;
            case GaitType::DIAGONAL:
                // Diagonal pattern
                phase_offsets_ = {0.0, 0.75, 0.5, 0.25};
                break;
            case GaitType::STAND:
                // Durma pozisyonu
                phase_offsets_ = {0.0, 0.0, 0.0, 0.0};
                duty_factor_ = 1.0;
                break;
        }
    }

    void update(double dt) {
        phase_ += dt * frequency_;
        if (phase_ >= 1.0) phase_ -= 1.0;
        
        // Her bacak için faz hesapla
        for (int i = 0; i < 4; i++) {
            leg_phases_[i] = std::fmod(phase_ + phase_offsets_[i], 1.0);
        }
    }

    bool isLegInStance(int leg_id) const {
        return leg_phases_[leg_id] < duty_factor_;
    }

    double getLegPhase(int leg_id) const {
        return leg_phases_[leg_id];
    }

    Eigen::Vector3d getFootTrajectory(int leg_id, const Eigen::Vector3d& nominal_pos) {
        double phase = leg_phases_[leg_id];
        
        if (isLegInStance(leg_id)) {
            // Stance fazı - ayak yerde, geriye doğru hareket
            double stance_progress = phase / duty_factor_;
            double x_offset = step_length_ * (0.5 - stance_progress);
            return nominal_pos + Eigen::Vector3d(x_offset, 0, 0);
        } else {
            // Swing fazı - ayak havada, öne doğru salınım
            double swing_progress = (phase - duty_factor_) / (1.0 - duty_factor_);
            double x_offset = step_length_ * (swing_progress - 0.5);
            double z_offset = step_height_ * std::sin(M_PI * swing_progress);
            return nominal_pos + Eigen::Vector3d(x_offset, 0, z_offset);
        }
    }

    void setStepParameters(double height, double length) {
        step_height_ = height;
        step_length_ = length;
    }

    void setFrequency(double freq) {
        frequency_ = freq;
    }

private:
    GaitType current_gait_;
    double phase_;
    double frequency_;
    double duty_factor_;
    double step_height_;
    double step_length_;
    std::array<double, 4> phase_offsets_;
    std::array<double, 4> leg_phases_;
};

class LegKinematics {
public:
    LegKinematics(double l1, double l2, double l3)
        : l1_(l1), l2_(l2), l3_(l3) {}

    // Inverse kinematics: Ayak pozisyonundan joint açılarına
    Eigen::Vector3d inverseKinematics(const Eigen::Vector3d& foot_pos) {
        double x = foot_pos.x();
        double y = foot_pos.y();
        double z = foot_pos.z();

        // Hip joint (yaw)
        double theta1 = std::atan2(y, x);

        // Düzlemsel IK (2D)
        double r = std::sqrt(x*x + y*y) - l1_;
        double d = std::sqrt(r*r + z*z);
        
        // Law of cosines
        double cos_theta3 = (d*d - l2_*l2_ - l3_*l3_) / (2*l2_*l3_);
        cos_theta3 = std::max(-1.0, std::min(1.0, cos_theta3));
        double theta3 = std::acos(cos_theta3);

        double alpha = std::atan2(z, r);
        double beta = std::acos((l2_*l2_ + d*d - l3_*l3_) / (2*l2_*d));
        double theta2 = alpha + beta;

        return Eigen::Vector3d(theta1, theta2, theta3);
    }

    // Forward kinematics: Joint açılarından ayak pozisyonuna
    Eigen::Vector3d forwardKinematics(const Eigen::Vector3d& joint_angles) {
        double theta1 = joint_angles[0];
        double theta2 = joint_angles[1];
        double theta3 = joint_angles[2];

        double x = (l1_ + l2_*std::cos(theta2) + l3_*std::cos(theta2 + theta3)) * std::cos(theta1);
        double y = (l1_ + l2_*std::cos(theta2) + l3_*std::cos(theta2 + theta3)) * std::sin(theta1);
        double z = l2_*std::sin(theta2) + l3_*std::sin(theta2 + theta3);

        return Eigen::Vector3d(x, y, z);
    }

private:
    double l1_, l2_, l3_;  // Link uzunlukları
};

class StateEstimator {
public:
    StateEstimator() {
        // Basit bir complementary filter
        accel_alpha_ = 0.98;
    }

    void updateFromIMU(const sensor_msgs::msg::Imu& imu_msg) {
        // Gyro verisi (güvenilir kısa vadede)
        Eigen::Vector3d gyro(
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        );

        // Accelerometer verisi (güvenilir uzun vadede)
        Eigen::Vector3d accel(
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        );

        // Quaternion'dan euler açıları
        Eigen::Quaterniond q(
            imu_msg.orientation.w,
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z
        );

        // Complementary filter ile orientation estimate
        // Gerçek uygulamada Kalman filter kullanılmalı
        estimated_orientation_ = q;
        estimated_angular_velocity_ = gyro;
    }

    Eigen::Quaterniond getOrientation() const {
        return estimated_orientation_;
    }

    Eigen::Vector3d getAngularVelocity() const {
        return estimated_angular_velocity_;
    }

private:
    double accel_alpha_;
    Eigen::Quaterniond estimated_orientation_;
    Eigen::Vector3d estimated_angular_velocity_;
};

class PavlovController : public rclcpp::Node {
public:
    PavlovController() : Node("pavlov_controller") {
        // Parametreleri declare et
        this->declare_parameter("control_frequency", 50.0);
        this->declare_parameter("gait_type", "trot");
        this->declare_parameter("step_height", 0.05);
        this->declare_parameter("step_length", 0.1);

        // Parametreleri oku
        double control_freq = this->get_parameter("control_frequency").as_double();
        std::string gait_type = this->get_parameter("gait_type").as_string();
        
        // Gait scheduler'ı başlat
        gait_scheduler_ = std::make_unique<GaitScheduler>();
        if (gait_type == "trot") {
            gait_scheduler_->setGaitType(GaitType::TROT);
        } else if (gait_type == "pace") {
            gait_scheduler_->setGaitType(GaitType::PACE);
        }

        double step_height = this->get_parameter("step_height").as_double();
        double step_length = this->get_parameter("step_length").as_double();
        gait_scheduler_->setStepParameters(step_height, step_length);

        // Kinematics başlat (örnek link uzunlukları)
        leg_kinematics_ = std::make_unique<LegKinematics>(0.05, 0.12, 0.12);

        // State estimator başlat
        state_estimator_ = std::make_unique<StateEstimator>();

        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&PavlovController::imuCallback, this, std::placeholders::_1)
        );

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&PavlovController::cmdVelCallback, this, std::placeholders::_1)
        );

        // Publishers
        joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_position_controller/commands", 10
        );

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10
        );

        // Control timer - 50Hz
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq));
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&PavlovController::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Pavlov Controller started at %f Hz", control_freq);
    }

private:
    void controlLoop() {
        auto current_time = this->now();
        last_update_time_ = this->now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Gait scheduler'ı güncelle
        gait_scheduler_->update(dt);

        // Her bacak için trajectory hesapla
        std::array<Eigen::Vector3d, 4> nominal_positions = {
            Eigen::Vector3d(0.15, 0.10, -0.20),   // Front left
            Eigen::Vector3d(0.15, -0.10, -0.20),  // Front right
            Eigen::Vector3d(-0.15, 0.10, -0.20),  // Rear left
            Eigen::Vector3d(-0.15, -0.10, -0.20)  // Rear right
        };

        // Desired velocity'den step parametrelerini ayarla
        double speed = std::sqrt(desired_velocity_.linear.x * desired_velocity_.linear.x +
                                desired_velocity_.linear.y * desired_velocity_.linear.y);
        gait_scheduler_->setStepParameters(0.05, speed * 0.2);

        // Joint komutlarını hesapla
        std::vector<double> joint_commands;
        joint_commands.reserve(12);

        for (int leg_id = 0; leg_id < 4; leg_id++) {
            // Ayak pozisyonu trajectory'sini al
            Eigen::Vector3d desired_foot_pos = 
                gait_scheduler_->getFootTrajectory(leg_id, nominal_positions[leg_id]);

            // Inverse kinematics ile joint açılarını hesapla
            Eigen::Vector3d joint_angles = 
                leg_kinematics_->inverseKinematics(desired_foot_pos);

            joint_commands.push_back(joint_angles[0]);
            joint_commands.push_back(joint_angles[1]);
            joint_commands.push_back(joint_angles[2]);
        }

        // Joint komutlarını publish et
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = joint_commands;
        joint_cmd_pub_->publish(msg);

        // Debug için joint state publish et
        publishJointState(joint_commands);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        state_estimator_->updateFromIMU(*msg);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        desired_velocity_ = *msg;
        
        // Hız komutuna göre gait frequency ayarla
        double speed = std::sqrt(msg->linear.x * msg->linear.x + 
                                msg->linear.y * msg->linear.y);
        gait_scheduler_->setFrequency(0.5 + speed);
    }

    void publishJointState(const std::vector<double>& positions) {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        
        // Joint isimleri
        std::vector<std::string> joint_names = {
            "front_left_hip", "front_left_shoulder", "front_left_knee",
            "front_right_hip", "front_right_shoulder", "front_right_knee",
            "rear_left_hip", "rear_left_shoulder", "rear_left_knee",
            "rear_right_hip", "rear_right_shoulder", "rear_right_knee"
        };

        msg.name = joint_names;
        msg.position = positions;
        
        joint_state_pub_->publish(msg);
    }

    // Members
    std::unique_ptr<GaitScheduler> gait_scheduler_;
    std::unique_ptr<LegKinematics> leg_kinematics_;
    std::unique_ptr<StateEstimator> state_estimator_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Twist desired_velocity_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PavlovController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}