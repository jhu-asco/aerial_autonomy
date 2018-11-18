#pragma once
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust_adaptive.h"
#include "aerial_autonomy/types/snap.h"
#include "rpyt_based_relative_pose_adaptive_estimate_controller_config.pb.h"

#include <Eigen/Dense>
#include <chrono>
#include <tf/tf.h>
#include <tuple>
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

/**
 * @brief A trajectory-tracking controller that estimates the mass parameter of
 * the quadrotor.
 * Implemented using RPTY control, but may use backstepping if things improve.
 */
class RPYTBasedRelativePoseAdaptiveEstimateController
    : public Controller<
          std::pair<double, ParticleState>,
          std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double>,
          RollPitchYawThrustAdaptive> {
public:
  /**
  * @brief Fixed Eigen vector 6x1
  */
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  /**
  * @brief Fixed Eigen matrix 6x3
  */
  using Matrix63d = Eigen::Matrix<double, 6, 3>;
  /**
  * @brief Fixed Eigen vector 6x1
  */
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  /**
  * @brief Fixed Eigen matrix 3x6
  */
  using Matrix36d = Eigen::Matrix<double, 3, 6>;
  /**
  * @brief Constructor
  */
  RPYTBasedRelativePoseAdaptiveEstimateController(
      RPYTBasedRelativePoseAdaptiveEstimateControllerConfig config) //,
      : config_(config),
        km(config_.km()),
        ag_(0, 0, -config_.acc_gravity()) {
    Eigen::Vector3d kp(config_.kp_xy(), config_.kp_xy(), config_.kp_z());
    Eigen::Vector3d kd(config_.kd_xy(), config_.kd_xy(), config_.kd_z());
    K_.leftCols<3>() = kp.asDiagonal();
    K_.rightCols<3>() = kd.asDiagonal();
    t0_ = std::chrono::high_resolution_clock::now();
  }
  /**
   * @brief Destructor
   */
  virtual ~RPYTBasedRelativePoseAdaptiveEstimateController() {}

protected:
  /**
   * @brief Run the control loop.
   *
   * @param sensor_data Current quad state and parameter estimate
   * @param goal Goal desired trajectory as a particle state and goal
   * @param control RPYTdM command
   * @return True if controller is successful in running
   */
  virtual bool runImplementation(
      std::pair<double, ParticleState> sensor_data,
      std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double> goal,
      RollPitchYawThrustAdaptive &control);
  /**
  * @brief Check if controller converged
  *
  * @param sensor_data Current quad state
  * @param goal Goal desired trajectory
  *
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(
      std::pair<double, ParticleState> sensor_data,
      std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double> goal);

private:
  /**
   * @brief Config that stores adaptive estimate config
   */
  RPYTBasedRelativePoseAdaptiveEstimateControllerConfig config_;
  /**
   * @brief The gain for adaptively estimating m
   */
  double km;
  /**
  * @brief Acceleration due to gravity
  */
  Eigen::Vector3d ag_;
  /**
  * @brief Gains for PD controller of second-order dynamics
  */
  Matrix36d K_;
  /**
  * @brief Time when the controller was created
  */
  std::chrono::time_point<std::chrono::high_resolution_clock> t0_;
};
