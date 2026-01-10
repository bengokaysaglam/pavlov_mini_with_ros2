/******************************************************************************
 * Pavlov Mini Robot Model for TOWR
 * 
 * Based on URDF parameters from pavlov_description package
 ******************************************************************************/

#ifndef PAVLOV_MINI_MODEL_H_
#define PAVLOV_MINI_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief Kinematic model for Pavlov Mini quadruped robot.
 * 
 * Parameters extracted from URDF:
 * - 4 legs (quadruped)
 * - Base dimensions: 0.26m x 0.16m x 0.09m
 * - Total leg length: ~0.20m (0.10m femur + 0.10m tibia)
 */
class PavlovMiniKinematicModel : public KinematicModel {
public:
  PavlovMiniKinematicModel() : KinematicModel(4)
  {
    // Nominal stance positions from URDF hip_ref joints
    // These are the positions where feet touch ground in default stance
    const double x_nominal_b = 0.0917;  // hip x-offset from base center
    const double y_nominal_b = 0.05355; // hip y-offset from base center
    const double z_nominal_b = -0.20;   // approximate total leg length (negative = below base)
    
    // LF = Left Front, RF = Right Front, LH = Left Hind, RH = Right Hind
    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;
    
    // Maximum deviation from nominal stance (conservative values for small robot)
    // x: forward/backward, y: left/right, z: up/down
    max_dev_from_nominal_ << 0.08, 0.05, 0.08;
  }
};

/**
 * @brief Dynamic model for Pavlov Mini quadruped robot.
 * 
 * Uses Single Rigid Body Dynamics (SRBD) model.
 * 
 * Parameters from URDF:
 * - Total mass: ~3.71 kg
 * - Base inertia tensor (from base_link):
 *   Ixx = 0.00372 kg⋅m²
 *   Iyy = 0.00868 kg⋅m²
 *   Izz = 0.00992 kg⋅m²
 *   Ixy = Ixz = Iyz = 0.0 (symmetric)
 */
class PavlovMiniDynamicModel : public SingleRigidBodyDynamics {
public:
  PavlovMiniDynamicModel()
    : SingleRigidBodyDynamics(
        3.71,      // mass (kg) - total robot mass from URDF
        0.00372,   // Ixx - roll inertia
        0.00868,   // Iyy - pitch inertia  
        0.00992,   // Izz - yaw inertia
        0.0,       // Ixy - cross term
        0.0,       // Ixz - cross term
        0.0,       // Iyz - cross term
        4          // number of end-effectors (4 legs)
      ) 
  {}
};

} // namespace towr

#endif /* PAVLOV_MINI_MODEL_H_ */