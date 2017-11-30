#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"

/**
 * @brief A controller that maps joystick commands to velocity goal
 * and commands rpythrust to the drone
 */
class JoystickVelocityController
    : public Controller<std::tuple<Joystick, VelocityYawRate, double>,
                        EmptyGoal, RollPitchYawRateThrust> {
public:
  /**
  * @brief Constructor
  *
  * @param rpyt_velocity_controller_config RPYT Controller config
  *
  * @param joystick_velocity_controller_config Joystick Velocity
  * Controller config
  *
  * @param controller_timer_duration in seconds
  *
  * @joystick mappings:
  * Channel 1 : X-velocity
  * Channel 2 : Y-velocity
  * Channel 3 : Z-velocity
  * Channel 4 : Yaw rate
  *
  */
  JoystickVelocityController(
      JoystickVelocityControllerConfig joystick_velocity_controller_config,
      double controller_timer_duration)
      : joystick_velocity_controller_config_(
            joystick_velocity_controller_config),
        rpyt_velocity_controller_(joystick_velocity_controller_config_
                                      .rpyt_based_velocity_controller_config(),
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
   * @brief Run the control loop.  Converts Joystick commands to Velocity.
   * @param sensor_data Joystick commands to be converted into Velocity yaw
   * rate.
   * Also provide current velocity yawrate and current yaw for rpyt controller
   * @param goal Goal is not used here
   * @param control RPYT to send to hardware
   * return True if successfully converted sensor data to control
   */
  virtual bool
  runImplementation(std::tuple<Joystick, VelocityYawRate, double> sensor_data,
                    EmptyGoal goal, RollPitchYawRateThrust &control);
  /**
  * @brief Converges when the internal controller converges
  *
  * @return Completed when controller converges
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<Joystick, VelocityYawRate, double> sensor_data,
      EmptyGoal goal);

private:
  /**
  * @brief Controller config for manual velocity controller
  */
  JoystickVelocityControllerConfig joystick_velocity_controller_config_;
  /**
  * @brief Internal controller to get rpyt from desired velocity
  */
  RPYTBasedVelocityController rpyt_velocity_controller_;
  /**
   * @brief convert joystick channels to velocity yaw rate
   *
   * @param joystick user provided channel information
   *
   * @return velocity and yaw rate obtained from joystick
   */
  VelocityYawRate convertJoystickToVelocityYawRate(const Joystick joystick);
};
