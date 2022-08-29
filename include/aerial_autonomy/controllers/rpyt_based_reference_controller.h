#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"

#include "rpyt_based_position_controller_config.pb.h"

#include <Eigen/Dense>
#include <chrono>
#include <glog/logging.h>

/**
 * @brief A position controller that gives rpyt commands
 */
template <class StateT, class ControlT>
class AbstractRPYTBasedReferenceController
    : public Controller<std::tuple<double, double, Velocity, PositionYaw>,
                        ReferenceTrajectoryPtr<StateT, ControlT>,
                        RollPitchYawRateThrust> {
public:
  /**
   * @brief Constructor
   *
   * @param config Controller config
   */
  AbstractRPYTBasedReferenceController(RPYTBasedPositionControllerConfig config)
      : config_(config) {
    resetPositionTolerance();
    // clang-format off
    DATA_HEADER("rpyt_reference_controller") << "Errorx"
                                             << "Errory"
                                             << "Errorz"
                                             << "Erroryaw"
                                             << "Errorvx"
                                             << "Errorvy"
                                             << "Errorvz"
                                             << "Cmd_roll"
                                             << "Cmd_pitch"
                                             << "Cmd_yawrate"
                                             << "Cmd_thrust" << DataStream::endl;
    // clang format on
  }
  void setPositionTolerance(PositionControllerConfig position_controller_config) {
    LOG(INFO)<<"Setting new position tolerance";
    position_controller_config_ = position_controller_config;
  }

  void resetPositionTolerance() {
    LOG(INFO)<<"Resetting position tolerance";
    position_controller_config_ = config_.velocity_based_position_controller_config().position_controller_config();
  }


protected:
  /**
   * @brief get reference states from reference trajectory state
   *
   * @param state_control_pair reference trajectory state control
   *
   * @return  positionyaw and velocity_yawrate
   */
  virtual std::pair<VelocityYawRate, PositionYaw>
  getReference(const StateT &state) = 0;

  /**
   * @brief Get feedforward acceleration
   *
   * @param state_control_pair input state control pair from reference
   *
   * @return feedforward acceleration
   */
  virtual Eigen::Vector3d
  getAcceleration(const std::pair<StateT, ControlT> &state_control_pair) {
    return Eigen::Vector3d(0, 0, 0);
  }

  /**
   * @brief Run rpyt reference controller
   *
   * @param sensor_data time since controller initialized, thrust_gain, current
   * velocity and position yaw
   * @param goal Goal reference trajectory
   * @param control control to send to quadrotor
   *
   * @return  true if succeeded in computing the control
   */
  bool runImplementation(
      std::tuple<double, double, Velocity, PositionYaw> sensor_data,
      ReferenceTrajectoryPtr<StateT, ControlT> goal,
      RollPitchYawRateThrust &control) {
    auto state_control_pair = goal->atTime(std::get<0>(sensor_data));
    auto simplified_goal = getReference(state_control_pair.first);
    double kt = std::get<1>(sensor_data);
    Velocity current_velocity = std::get<2>(sensor_data);
    PositionYaw current_position_yaw = std::get<3>(sensor_data);
    PositionYaw error_position_yaw =
        simplified_goal.second - current_position_yaw;
    Velocity error_velocity =
        (Velocity)simplified_goal.first - current_velocity;
    Eigen::Vector3d desired_acceleration;
    // get gains
    auto &velocity_config = config_.rpyt_based_velocity_controller_config();
    auto &velocity_position_config =
        config_.velocity_based_position_controller_config();
    desired_acceleration[0] =
        velocity_position_config.position_gain() * error_position_yaw.x +
        velocity_config.kp_xy() * error_velocity.x;
    desired_acceleration[1] =
        velocity_position_config.position_gain() * error_position_yaw.y +
        velocity_config.kp_xy() * error_velocity.y;
    desired_acceleration[2] =
        velocity_position_config.z_gain() * error_position_yaw.z +
        velocity_config.kp_z() * error_velocity.z;
    ///\todo Add feedforward acceleration from reference
    double magnitude_acceleration = desired_acceleration.norm();
    if (magnitude_acceleration > velocity_config.max_acc_norm()) {
      desired_acceleration =
          (velocity_config.max_acc_norm() / magnitude_acceleration) *
          desired_acceleration;
    }
    // desired_acceleration[2] = desired_acceleration[2] + gravity_;
    // Add feedforward acc
    desired_acceleration =
        desired_acceleration + getAcceleration(state_control_pair);
    std::pair<double, double> rp = conversions::accelerationToRollPitch(
        current_position_yaw.yaw, desired_acceleration,
        velocity_config.tolerance_rp());
    control.r = math::clamp(rp.first, -velocity_config.max_rp(),
                            velocity_config.max_rp());
    control.p = math::clamp(rp.second, -velocity_config.max_rp(),
                            velocity_config.max_rp());
    control.y = simplified_goal.first.yaw_rate +
                velocity_position_config.yaw_gain() * error_position_yaw.yaw;
    control.y = math::clamp(control.y, -velocity_position_config.max_yaw_rate(),
                            velocity_position_config.max_yaw_rate());
    control.t = desired_acceleration.norm() / kt;
    control.t = math::clamp(control.t, velocity_config.min_thrust(),
                            velocity_config.max_thrust());
    DATA_LOG("rpyt_reference_controller")
        << error_position_yaw.x << error_position_yaw.y << error_position_yaw.z
        << error_position_yaw.yaw << error_velocity.x << error_velocity.y
        << error_velocity.z << control.r << control.p << control.y << control.t << DataStream::endl;
    return true;
  }
  /**
  * @brief Check if rpyt based position controller converged
  *
  * @param sensor_data Current position yaw and velocity
  * @param goal Goal position yaw
  *
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<double, double, Velocity, PositionYaw> sensor_data,
      ReferenceTrajectoryPtr<StateT, ControlT> goal) {
    ControllerStatus status = ControllerStatus::Status::Active;
    auto simplified_goal = getReference(goal->goal(std::get<0>(sensor_data)));
    Velocity current_velocity = std::get<2>(sensor_data);
    PositionYaw current_position_yaw = std::get<3>(sensor_data);
    PositionYaw error_position_yaw =
        simplified_goal.second - current_position_yaw;
    Velocity error_velocity =
        (Velocity)simplified_goal.first - current_velocity;
    // Check position and goal tolerance
    auto &velocity_config = config_.rpyt_based_velocity_controller_config();
    //auto &velocity_position_config =
    //    config_.velocity_based_position_controller_config();
    //auto &position_controller_config =
    //    velocity_position_config.position_controller_config();
    PositionControllerConfig position_controller_config = position_controller_config_;
    auto &tolerance_vel =
        velocity_config.velocity_controller_config().goal_velocity_tolerance();
    const config::Position &tolerance_pos =
        position_controller_config.goal_position_tolerance();
    const double &tolerance_yaw =
        position_controller_config.goal_yaw_tolerance();
    const bool &check_yaw_continuously =
        position_controller_config.check_yaw_continuously();
    const double &tolerance_continuous_yaw =
        position_controller_config.continuous_yaw_tolerance();
    const bool &check_z_continuously =
        position_controller_config.check_z_continuously();
    const double &tolerance_continuous_z_min =
        position_controller_config.continuous_z_min_tolerance();
    const double &tolerance_continuous_z_max =
        position_controller_config.continuous_z_max_tolerance();
    bool new_warning = false;
    // Compare
    if (std::abs(error_position_yaw.x) <= tolerance_pos.x() &&
        std::abs(error_position_yaw.y) <= tolerance_pos.y() &&
        std::abs(error_position_yaw.z) <= tolerance_pos.z() &&
        std::abs(error_position_yaw.yaw) <= tolerance_yaw &&
        std::abs(error_velocity.x) < tolerance_vel.vx() &&
        std::abs(error_velocity.y) < tolerance_vel.vy() &&
        std::abs(error_velocity.z) < tolerance_vel.vz()) {
      VLOG_EVERY_N(1, 100) << "Reached goal";
      status.setStatus(ControllerStatus::Completed, "Reached goal");
    }
    if (check_yaw_continuously) {
      if (std::abs(error_position_yaw.yaw) > tolerance_continuous_yaw) {
        std::string warning_description = "Yaw error critical: " + std::to_string(error_position_yaw.yaw);
        status.setWarning(true, warning_description);
        new_warning = true;
      }
    }
    if (check_z_continuously) {
      if ((error_position_yaw.z < tolerance_continuous_z_min) || 
          (error_position_yaw.z > tolerance_continuous_z_max)) {
        std::string warning_description = "Z error critical: " + std::to_string(error_position_yaw.yaw);
        status.setWarning(true, warning_description);
        new_warning = true;
      }
    }
    if (!new_warning)
    {
      // Set back to false if no new warning was generated
      status.setWarning(false);
    }
    status<<"Errors: "<<error_position_yaw.x<<error_position_yaw.y<<error_position_yaw.z<<error_position_yaw.yaw;
    return status;
  }

private:
  RPYTBasedPositionControllerConfig config_; ///< Gains for reference tracking
  Atomic<PositionControllerConfig> position_controller_config_;/// position controller config
};

class RPYTBasedReferenceControllerEigen
    : public AbstractRPYTBasedReferenceController<Eigen::VectorXd,
                                                  Eigen::VectorXd> {
public:
  using BaseClass =
      AbstractRPYTBasedReferenceController<Eigen::VectorXd, Eigen::VectorXd>;
  using BaseClass::BaseClass; // Inherit constructor
protected:
  const double gravity_ = 9.81; /// Magnitude of gravity
  std::pair<VelocityYawRate, PositionYaw>
  getReference(const Eigen::VectorXd &state);
  Eigen::Vector3d getAcceleration(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd> &state_control_pair);
};
