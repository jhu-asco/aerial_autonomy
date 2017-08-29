#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"

/**
 * @brief A controller that passes joystick commands to a drone's RPYT
 * controller
 */
class JoystickVelocityController
    : public Controller<std::tuple<Joystick, VelocityYaw>, EmptyGoal,
                        RollPitchYawThrust> {
public:
  /**
  *
  */
  JoystickVelocityController(
      RPYTBasedVelocityControllerConfig &rpyt_velocity_controller_config,
      JoystickVelocityControllerConfig &joystick_velocity_controller_config)
      : rpyt_velocity_controller_config_(rpyt_velocity_controller_config),
        rpyt_velocity_controller_(rpyt_velocity_controller_config_),
        joystick_velocity_controller_config_(joystick_velocity_controller_config) {}

protected:
  /**
   * @brief Run the control loop.  Converts Joystick commands to Velocity.
   * @param sensor_data Joystick commands to be converted into Velocity.
   * @param goal Goal is not used here
   * @param control Velocity to send to hardware
   * return True if successfully converted sensor data to control
   */
  virtual bool runImplementation(std::tuple<Joystick, VelocityYaw> sensor_data,
                                 EmptyGoal goal, RollPitchYawThrust &control);
  /**
  * @brief Default implementation since there is no concept of convergence
  * for manual rpyt controller
  * @return True always
  */
  virtual ControllerStatus
  isConvergedImplementation(std::tuple<Joystick, VelocityYaw> sensor_data,
                            EmptyGoal goal) {
    return ControllerStatus::Completed;
  }

private:
  /**
  * @ Config for rpyt velocity controller
  */
  RPYTBasedVelocityControllerConfig &rpyt_velocity_controller_config_;
  /**
  * @brief Internal controller to get rpyt from desired velocity
  */
  RPYTBasedVelocityController rpyt_velocity_controller_;
  /**
  * @brief Controller config for manual velocity controller
  */
  JoystickVelocityControllerConfig joystick_velocity_controller_config_;
};
