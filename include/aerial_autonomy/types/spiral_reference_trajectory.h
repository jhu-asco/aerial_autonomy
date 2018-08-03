#pragma once
#include "aerial_autonomy/types/reference_trajectory.h"
#include "arm_sine_controller_config.pb.h"
#include "spiral_reference_trajectory_config.pb.h"
#include <Eigen/Dense>
#include <memory>

/**
* @brief Spiral reference trajectory for Aerial manipulation system
*/
class SpiralReferenceTrajectory
    : public ReferenceTrajectory<Eigen::VectorXd, Eigen::VectorXd> {
public:
  /**
  * @brief State type
  */
  using StateT = Eigen::VectorXd;
  /**
  * @brief Control type
  */
  using ControlT = Eigen::VectorXd;
  /**
  * @brief Constructor
  *
  * @param config reference trajectory parameters
  * @param arm_config Arm reference trajectory parameters
  * @param current_position Current quadrotor position
  * @param current_yaw Current quadrotor yaw
  */
  SpiralReferenceTrajectory(SpiralReferenceTrajectoryConfig config,
                            ArmSineControllerConfig arm_config,
                            Eigen::Vector3d current_position,
                            double current_yaw);
  /**
  * @brief Get the state and control at specified time
  *
  * @param t current time
  *
  * @return state and control at current time
  */
  std::pair<StateT, ControlT> atTime(double t) const;
  /**
  * @brief Compute roll and pitch necessary to align z axis along specified axis
  *
  * @param roll Desired roll
  * @param pitch Desired pitch
  * @param yaw Current yaw
  * @param acceleration_vector acceleration vector in inertial frame
  */
  void getRP(double &roll, double &pitch, double yaw,
             Eigen::Vector3d acceleration_vector) const;
  /**
  * @brief Get current acceleration along spiral trajectory
  *
  * @param angle The angle along the circle in the spiral
  * @param omega_squared The square of angular velocity along the spiral
  * @param rx x radius of the spiral
  * @param ry y radius of the spiral
  *
  * @return acceleration vector in inertial frame
  */
  Eigen::Vector3d getAcceleration(double angle, double omega_squared, double rx,
                                  double ry) const;

private:
  SpiralReferenceTrajectoryConfig
      config_; ///< Spiral reference trajectory parameters
  ArmSineControllerConfig arm_config_; ///< Arm reference trajectory parameters
  Eigen::Vector3d current_position_;   ///< Current quadrotor position
  double current_yaw_;                 ///< Current quadrotor yaw
  static constexpr double epsilon = 1e-6; ///< Tolerance for rp singularity
  static constexpr double gravity_magnitude_ = 9.81; ///< Gravity magnitude
};
/**
 * @brief shared pointer to spiral reference trajectory
 */
using SpiralReferenceTrajectoryPtr = std::shared_ptr<SpiralReferenceTrajectory>;
