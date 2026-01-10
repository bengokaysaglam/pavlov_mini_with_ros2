#include <array>
#include <cmath>
#include <Eigen/Dense>

class GaitScheduler
{
public:
    enum class GaitType {
        TROT,
        PACE,
        DIAGONAL,
        BOUND
    };
    
    GaitScheduler() {
        loadGaitParameters();
    }
    
    void update(double dt) {
        phase_ += dt * frequency_;
        if (phase_ >= 1.0) phase_ -= 1.0;
        
        // updateLegPhases();
    }
    
    bool isLegInStance(int leg_id) {
        return leg_phases_[leg_id] < duty_factor_;
    }
    
    Eigen::Vector3d getFootTrajectory(int leg_id) {
        // Swing/stance fazına göre ayak pozisyonu hesapla
        if (isLegInStance(leg_id)) {
            return computeStanceTrajectory(leg_id);
        } else {
            return computeSwingTrajectory(leg_id);
        }
    }

private:
    void loadGaitParameters() {
        // YAML'dan yükle
        // trot: duty_factor = 0.5, phase_offsets = [0, 0.5, 0.5, 0]
        // pace: duty_factor = 0.5, phase_offsets = [0, 0.5, 0, 0.5]
    }

    // Şimdilik koydum boş olmasın ve error almayayım diye.
    Eigen::Vector3d computeStanceTrajectory(int leg_id)
    {
        return Eigen::Vector3d(0,0,0);
    }

    Eigen::Vector3d computeSwingTrajectory(int leg_id)
    {
        return Eigen::Vector3d(0,0,0);
    }
    
    double phase_ = 0.0;
    double frequency_ = 1.0;
    double duty_factor_ = 0.5;
    std::array<double, 4> leg_phases_;
    std::array<double, 4> phase_offsets_;
    GaitType current_gait_;
};