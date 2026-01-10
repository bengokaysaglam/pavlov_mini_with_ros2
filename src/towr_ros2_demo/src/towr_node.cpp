#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// TOWR includes
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>

// Pavlov Mini model - bu dosyayı aynı dizine koyacağız
#include "../include/towr_ros2_demo/pavlov_mini_model.h"

class PavlovTowrNode : public rclcpp::Node
{
public:
  PavlovTowrNode() : Node("pavlov_towr_node")
  {
    RCLCPP_INFO(this->get_logger(), "🤖 Pavlov Mini TOWR Node başlatıldı!");
    
    // Publisher oluştur
    publisher_ = this->create_publisher<std_msgs::msg::String>("towr_trajectory", 10);
    
    // Timer ile periyodik olarak trajektori hesapla (10 saniyede bir)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&PavlovTowrNode::computeTrajectory, this));
      
    RCLCPP_INFO(this->get_logger(), 
                "=================================================");
    RCLCPP_INFO(this->get_logger(), 
                "  Pavlov Mini - TOWR Trajectory Optimization");
    RCLCPP_INFO(this->get_logger(), 
                "  Kütle: 3.71 kg | 4 Bacak | Bacak uzunluğu: 20cm");
    RCLCPP_INFO(this->get_logger(), 
                "=================================================");
  }

private:
  void computeTrajectory()
  {
    RCLCPP_INFO(this->get_logger(), "🚀 Trajektori hesaplanıyor...");
    
    try {
      // TOWR formülasyonu oluştur
      towr::NlpFormulation formulation;
      
      // Terrain (düz zemin)
      formulation.terrain_ = std::make_shared<towr::FlatGround>(0.0);
      
      // Pavlov Mini robot modelini kullan
      formulation.model_.kinematic_model_ = std::make_shared<towr::PavlovMiniKinematicModel>();
      formulation.model_.dynamic_model_   = std::make_shared<towr::PavlovMiniDynamicModel>();
      
      // Başlangıç gövde pozisyonu (daha yüksek başlayalım)
      formulation.initial_base_.lin.at(towr::kPos) << 0.0, 0.0, 0.30;
      
      // Hedef gövde pozisyonu (daha kısa mesafe)
      formulation.final_base_.lin.at(towr::kPos) << 0.3, 0.0, 0.30;
      
      // 4 bacak için başlangıç pozisyonları (zemin seviyesinde, nominal stance)
      // URDF'den gelen hip pozisyonları + bacak uzunluğu
      formulation.initial_ee_W_.push_back(Eigen::Vector3d( 0.0917,  0.05355, 0.0)); // LF
      formulation.initial_ee_W_.push_back(Eigen::Vector3d( 0.0917, -0.05355, 0.0)); // RF
      formulation.initial_ee_W_.push_back(Eigen::Vector3d(-0.0917,  0.05355, 0.0)); // LH
      formulation.initial_ee_W_.push_back(Eigen::Vector3d(-0.0917, -0.05355, 0.0)); // RH
      
      // Basit walking gait (tüm bacaklar sırayla)
      // Daha uzun stance fazları, daha stabil
      formulation.params_.ee_phase_durations_.push_back({0.5, 0.2}); // LF
      formulation.params_.ee_phase_durations_.push_back({0.5, 0.2}); // RF
      formulation.params_.ee_phase_durations_.push_back({0.5, 0.2}); // LH
      formulation.params_.ee_phase_durations_.push_back({0.5, 0.2}); // RH
      
      // Tüm bacaklar başlangıçta yerde
      formulation.params_.ee_in_contact_at_start_.push_back(true);  // LF
      formulation.params_.ee_in_contact_at_start_.push_back(true);  // RF
      formulation.params_.ee_in_contact_at_start_.push_back(true);  // LH
      formulation.params_.ee_in_contact_at_start_.push_back(true);  // RH
      
      // Optimizasyon problemi oluştur
      ifopt::Problem nlp;
      towr::SplineHolder solution;
      
      // Variables ekle
      for (auto c : formulation.GetVariableSets(solution)) {
        nlp.AddVariableSet(c);
      }
      
      // Constraints ekle
      for (auto c : formulation.GetConstraints(solution)) {
        nlp.AddConstraintSet(c);
      }
      
      // Costs ekle
      for (auto c : formulation.GetCosts()) {
        nlp.AddCostSet(c);
      }
      
      // IPOPT solver ile çöz
      auto solver = std::make_shared<ifopt::IpoptSolver>();
      solver->SetOption("jacobian_approximation", "exact");
      solver->SetOption("max_cpu_time", 20.0);
      solver->Solve(nlp);
      
      // Sonucu yayınla
      auto message = std_msgs::msg::String();
      message.data = "✅ Pavlov Mini trajektori hesaplandı! Hedef: (0.3, 0.0, 0.30)";
      publisher_->publish(message);
      
      RCLCPP_INFO(this->get_logger(), "✅ Trajektori hesaplama başarılı!");
      
      // Toplam süreyi hesapla
      double total_time = 0.0;
      for (double duration : formulation.params_.ee_phase_durations_[0]) {
        total_time += duration;
      }
      
      RCLCPP_INFO(this->get_logger(), 
                  "📊 Toplam hareket süresi: %.2f saniye", total_time);
      RCLCPP_INFO(this->get_logger(), 
                  "🐕 Gait: Walking (yürüme)");
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), 
                   "❌ Hata: %s", e.what());
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PavlovTowrNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}