#pragma once
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/particle_state_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust_adaptive.h"
#include "aerial_autonomy/types/snap.h"
#include "rpyt_based_relative_pose_adaptive_estimate_controller_config.pb.h"

#include "aerial_autonomy/log/log.h"
#include <Eigen/Dense>
#include <chrono>
#include <tf/tf.h>
#include <tuple>

#include <glog/logging.h>

/**
 * @brief A trajectory-tracking controller that estimates the mass parameter of
 * the quadrotor.
 * Implemented using RPTY control, but may use backstepping if things improve.
 */
class RPYTBasedRelativePoseAdaptiveEstimateController
    : public Controller<
          std::tuple<double, double, ParticleState>,
          ReferenceTrajectoryPtr<ParticleStateYaw, Snap>,
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
      RPYTBasedRelativePoseAdaptiveEstimateControllerConfig config)
      : config_(config), km_(config_.km()), ag_(0, 0, -config_.acc_gravity()) {
    DATA_HEADER("adaptive_controller") << "mhat"
                                       << "lyap"
                                       << "delta_px"
                                       << "delta_py"
                                       << "delta_pz"
                                       << "delta_vx"
                                       << "delta_vy"
                                       << "delta_vz"
                                       << "yaw_cmd"
                                       << "x_goal"
                                       << "y_goal"
                                       << "z_goal"
                                       << "x"
                                       << "y"
                                       << "z"
                                       << "acc_x"
                                       << "acc_y"
                                       << "acc_z"
                                       << "roll"
                                       << "pitch" << DataStream::endl;
    Eigen::Vector3d kp(config_.kp_xy(), config_.kp_xy(), config_.kp_z());
    Eigen::Vector3d kd(config_.kd_xy(), config_.kd_xy(), config_.kd_z());
    K_.leftCols<3>() = kp.asDiagonal();
    K_.rightCols<3>() = kd.asDiagonal();
    t0_ = std::chrono::high_resolution_clock::now();
    P_.topLeftCorner<3, 3>() = kp.asDiagonal();
    Eigen::Vector3d mI(
        6, 6, 6); // Assumption of m_hat for estimating lyapanov function
    P_.bottomRightCorner<3, 3>() = mI.asDiagonal();
    P_.topRightCorner<3, 3>() = config_.eps() * mI.asDiagonal();
    P_.bottomLeftCorner<3, 3>() = config_.eps() * mI.asDiagonal();
  }
  /**
   * @brief Destructor
   */
  virtual ~RPYTBasedRelativePoseAdaptiveEstimateController() {}

  /**
  * @brief Set goal
  *
  * @param goal new controller goal
  */
  void
  setGoal(ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal) {
    t0_ = std::chrono::high_resolution_clock::now();
    Controller<std::tuple<double, double, ParticleState>,
               ReferenceTrajectoryPtr<ParticleStateYaw, Snap>,
               RollPitchYawThrustAdaptive>::setGoal(goal);
  }

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
      std::tuple<double, double, ParticleState> sensor_data,
      ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal,
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
      std::tuple<double, double, ParticleState> sensor_data,
      ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal);

private:
  /**
   * @brief Config that stores adaptive estimate config
   */
  RPYTBasedRelativePoseAdaptiveEstimateControllerConfig config_;
  /**
   * @brief The gain for adaptively estimating m
   */
  double km_;
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

  /**
  * @brief P matrix for the estimated lyapunov functon
  */
  Matrix6d P_;
};
