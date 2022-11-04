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
      : config_(config), cumulative_error_(0, 0, 0, 0), dt_(std::chrono::milliseconds(20)) {
    resetPositionTolerance();
    // clang-format off
    auto &velocity_position_config =
        config_.velocity_based_position_controller_config();
    CHECK(velocity_position_config.position_i_gain() >= 0) << "Gain should be non-negative";
    CHECK(velocity_position_config.z_i_gain() >= 0) << "Gain should be non-negative";
    CHECK(velocity_position_config.yaw_i_gain() >= 0) << "Gain should be non-negative";
    CHECK(velocity_position_config.position_saturation_value() >= 0)
        << "Saturation value should be non-negative";
    CHECK(velocity_position_config.z_saturation_value() >= 0)
        << "Saturation value should be non-negative";
    CHECK(velocity_position_config.yaw_saturation_value() >= 0)
        << "Saturation value should be non-negative";
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
                                             << "Cmd_thrust" 
                                             << "Cumulative_Errorx"
                                             << "Cumulative_Errory"
                                             << "Cumulative_Errorz"
                                             << "Cumulative_Erroryaw" 
                                             << "Goalx"
                                             << "Goaly" 
                                             << "Goalz"
                                             << "Goalyaw"
                                             << DataStream::endl;
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
   * @brief If the command (p_command + integrator) is saturated, the function
   * resets the integrator to ensure p_command + integrator = saturation.
   *
   * @param integrator
   * @param p_command
   * @param saturation
   *
   * @return the command after resetting integrator
   */
  inline double backCalculate(double &integrator, const double &p_command,
                              const double &saturation,
                              const double &integrator_saturation_value) {
    double command = p_command + integrator;
    double command_out = math::clamp(command, -saturation, saturation);
    if (command > saturation) {
      LOG(WARNING) << "Above saturation_gain, setting integrator to -saturation_value";
      integrator = -integrator_saturation_value;
    } else if (command < -saturation) {
      LOG(WARNING) << "Below -saturation_gain, setting integrator to saturation_value";
      integrator = integrator_saturation_value;
    }
    return command_out;
  }

  /**
   * @brief Getter for internal cumulative error stored
   *
   * @return cumulative position_yaw error multiplied by dt and i gain
   */
  PositionYaw getCumulativeError() const { return cumulative_error_; }

  /**
   * @brief Set the goal and optionally reset the controller
   * @param goal The goal to set
   * @param reset Resets controller integrator if true
   * \todo (Matt) Remove reset from setGoal and create separate reset function
   * in base Controller
   */
  void setGoal(PositionYaw goal, bool reset = true) {
    Controller<std::tuple<double, double, Velocity, PositionYaw>,
                        ReferenceTrajectoryPtr<StateT, ControlT>,
                        RollPitchYawRateThrust>::setGoal(goal);
    if (reset)
      resetIntegrator();
  }

  /**
   * @brief Set the goal and reset the controller
   * @param goal The goal to set
   */
  void setGoal(PositionYaw goal) {
    Controller<std::tuple<double, double, Velocity, PositionYaw>,
                        ReferenceTrajectoryPtr<StateT, ControlT>,
                        RollPitchYawRateThrust>::setGoal(goal);
    resetIntegrator();
  }

  /**
   * @brief Compute the integrator internally based on
   * back calculation
   */
  void resetIntegrator() {
    VLOG(1) << "Reseting integrator";
    cumulative_error_ = PositionYaw(0, 0, 0, 0);
  }

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

    // P gains
    PositionYaw p_position_diff(error_position_yaw.x * velocity_position_config.position_gain(),
                            error_position_yaw.y * velocity_position_config.position_gain(),
                            error_position_yaw.z * velocity_position_config.z_gain(),
                            error_position_yaw.yaw * velocity_position_config.yaw_gain());

    // D gains
    PositionYaw d_velocity_diff(velocity_config.kp_xy() * error_velocity.x,
                                velocity_config.kp_xy() * error_velocity.y,
                                velocity_config.kp_z() * error_velocity.z,
                                0);

    // Integrator
    PositionYaw i_position_diff(error_position_yaw.x * velocity_position_config.position_i_gain(),
                                error_position_yaw.y * velocity_position_config.position_i_gain(),
                                error_position_yaw.z * velocity_position_config.z_i_gain(),
                                error_position_yaw.yaw * velocity_position_config.yaw_i_gain());
    
    // Determine dt_ (otherwise could set constant)
    std::chrono::time_point<std::chrono::high_resolution_clock> current_time =
      std::chrono::high_resolution_clock::now();
    dt_ = std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                last_run_time_);
    // If dt is high (such as in first step) set back low 
    if (dt_ > std::chrono::milliseconds(100))
    {
      dt_ = std::chrono::milliseconds(20);
    }
    last_run_time_ = std::chrono::system_clock::now();
    cumulative_error_ = cumulative_error_ + i_position_diff * dt_.count();

    desired_acceleration[0] = backCalculate(cumulative_error_.x, p_position_diff.x + d_velocity_diff.x,
                                            velocity_position_config.max_acceleration(), 
                                            velocity_position_config.position_saturation_value());
    desired_acceleration[1] = backCalculate(cumulative_error_.y, p_position_diff.y + d_velocity_diff.y,
                                            velocity_position_config.max_acceleration(), 
                                            velocity_position_config.position_saturation_value());
    desired_acceleration[2] = backCalculate(cumulative_error_.z, p_position_diff.z + d_velocity_diff.z,
                                            velocity_position_config.max_acceleration(), 
                                            velocity_position_config.z_saturation_value());

    // desired_acceleration[0] =
    //     velocity_position_config.position_gain() * error_position_yaw.x +
    //     velocity_config.kp_xy() * error_velocity.x;
    // desired_acceleration[1] =
    //     velocity_position_config.position_gain() * error_position_yaw.y +
    //     velocity_config.kp_xy() * error_velocity.y;
    // desired_acceleration[2] =
    //     velocity_position_config.z_gain() * error_position_yaw.z +
    //     velocity_config.kp_z() * error_velocity.z;
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
    control.y = simplified_goal.first.yaw_rate + p_position_diff.yaw;
                // velocity_position_config.yaw_gain() * error_position_yaw.yaw;
    // control.y = math::clamp(control.y, -velocity_position_config.max_yaw_rate(),
    //                         velocity_position_config.max_yaw_rate());
    control.y = backCalculate(cumulative_error_.yaw, control.y,
                              velocity_position_config.max_yaw_rate(),
                              velocity_position_config.yaw_saturation_value());
    control.t = desired_acceleration.norm() / kt;
    control.t = math::clamp(control.t, velocity_config.min_thrust(),
                            velocity_config.max_thrust());
    PositionYaw overall_goal = getReference(goal->goal(std::get<0>(sensor_data))).second;
    DATA_LOG("rpyt_reference_controller")
        << error_position_yaw.x << error_position_yaw.y << error_position_yaw.z
        << error_position_yaw.yaw << error_velocity.x << error_velocity.y
        << error_velocity.z << control.r << control.p << control.y << control.t 
        << cumulative_error_.x << cumulative_error_.y << cumulative_error_.z << cumulative_error_.yaw 
        << overall_goal.x << overall_goal.y << overall_goal.z << overall_goal.yaw
        << DataStream::endl;
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
    const double &continuous_region_x_tolerance =
        position_controller_config.continuous_region_x_tolerance();
    const double &continuous_region_y_tolerance =
        position_controller_config.continuous_region_y_tolerance();
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
    // If close to goal, check continuous thresholds
    if ((std::abs(error_position_yaw.x) < continuous_region_x_tolerance) && 
        (std::abs(error_position_yaw.y) < continuous_region_y_tolerance))
    {
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
          std::string warning_description = "Z error critical: " + std::to_string(error_position_yaw.z);
          status.setWarning(true, warning_description);
          new_warning = true;
        }
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
  PositionYaw cumulative_error_; ///< Error integrated over multiple runs
  std::chrono::duration<double>
      dt_; ///< Time diff between different successive runImplementation calls
  std::chrono::time_point<std::chrono::high_resolution_clock> last_run_time_ =
      std::chrono::high_resolution_clock::now();
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
