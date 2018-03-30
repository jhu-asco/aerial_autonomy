#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "aerial_autonomy/controllers/velocity_based_position_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "rpyt_based_position_controller_config.pb.h"

#include <chrono>

/**
 * @brief A position controller that gives rpyt commands
 */
class RPYTBasedPositionController
    : public Controller<std::tuple<VelocityYawRate, PositionYaw>, PositionYaw,
                        RollPitchYawRateThrust> {
public:
  /**
   * @brief Constructor
   *
   * Instantiates a rpyt based velocity controller and a
   * velocity based position controller.
   *
   * @param config A config containing position based vel controller and
   * velocity
   *               based position controller
   * @param controller_timer_duration The time difference between calls for
   * internal controllers
   */
  RPYTBasedPositionController(
      RPYTBasedPositionControllerConfig config,
      std::chrono::duration<double> controller_timer_duration)
      : rpyt_velocity_controller_(
            config.rpyt_based_velocity_controller_config(),
            controller_timer_duration),
        position_controller_(config.velocity_based_position_controller_config(),
                             controller_timer_duration) {}
  /**
  * @brief Update RPYT controller config
  */
  void updateRPYTConfig(RPYTBasedVelocityControllerConfig &config) {
    rpyt_velocity_controller_.updateConfig(config);
  }
  /**
  * @brief get RPYT controller config
  */
  RPYTBasedVelocityControllerConfig getRPYTConfig() {
    return rpyt_velocity_controller_.getConfig();
  }

protected:
  /**
   * @brief Run the control loop.  Uses a rpyt controller to achieve the
   * desired position.
   * @param sensor_data Current position and velocity
   * @param goal Goal position
   * @param control RPYT command to send to hardware
   * @return true if command to reach goal is found
   */
  virtual bool
  runImplementation(std::tuple<VelocityYawRate, PositionYaw> sensor_data,
                    PositionYaw goal, RollPitchYawRateThrust &control);
  /**
  * @brief Check if rpyt based position controller converged
  *
  * @param sensor_data Current position yaw and velocity
  * @param goal Goal position yaw
  *
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<VelocityYawRate, PositionYaw> sensor_data, PositionYaw goal);

private:
  /**
   * @brief Controller that specifies rpyt commands to achieve a desired
   * velocity
   */
  RPYTBasedVelocityController rpyt_velocity_controller_;
  /**
   * @brief Controller that specifies desired velocity to achieve a position
   */
  VelocityBasedPositionController position_controller_;
};
