#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"

/**
 * @brief A controller that maps joystick commands to velocity goal
 * and commands rpythrust to the drone
 */
class JoystickVelocityController
    : public Controller<std::tuple<Joystick, VelocityYaw>, EmptyGoal,
                        RollPitchYawThrust> {
public:
  /**
  * @brief Constructor
  *
  * @param rpyt_velocity_controller_config RPYT Controller config
  *
  * @param joystick_velocity_controller_config Joystick Velocity
  * Controller config
  *
  */
  JoystickVelocityController(
      RPYTBasedVelocityControllerConfig &rpyt_velocity_controller_config,
      JoystickVelocityControllerConfig &joystick_velocity_controller_config,
      double controller_timer_duration)
      : controller_timer_duration_(controller_timer_duration),
        rpyt_velocity_controller_config_(rpyt_velocity_controller_config),
        rpyt_velocity_controller_(rpyt_velocity_controller_config_,
                                  controller_timer_duration_),
        joystick_velocity_controller_config_(
            joystick_velocity_controller_config) {
    DATA_HEADER("joystick_velocity_controller") << "Joy1"
                                                << "Joy2"
                                                << "Joy3"
                                                << "Joy4"
                                                << "velocity_x"
                                                << "velocity_y"
                                                << "velocity_z"
                                                << "Yaw" << DataStream::endl;
  }

protected:
  /**
   * @brief Run the control loop.  Converts Joystick commands to Velocity.
   * @param sensor_data Joystick commands to be converted into Velocity.
   * @param goal Goal is not used here
   * @param control RPYT to send to hardware
   * return True if successfully converted sensor data to control
   */
  virtual bool runImplementation(std::tuple<Joystick, VelocityYaw> sensor_data,
                                 EmptyGoal goal, RollPitchYawThrust &control);
  /**
  * @brief Converges when the internal controller converges
  *
  * @return Completed when controller converges
  */
  virtual ControllerStatus
  isConvergedImplementation(std::tuple<Joystick, VelocityYaw> sensor_data,
                            EmptyGoal goal) {
    Joystick joy_data = std::get<0>(sensor_data);
    VelocityYaw velocity_data = std::get<1>(sensor_data);

    DATA_LOG("joystick_velocity_controller")
        << joy_data.channel1 << joy_data.channel2 << joy_data.channel3
        << joy_data.channel4 << velocity_data.x << velocity_data.y
        << velocity_data.z << velocity_data.yaw << DataStream::endl;

    return rpyt_velocity_controller_.isConverged(velocity_data);
  }

private:
  /**
  * @brief Controller timestep
  */
  double controller_timer_duration_;
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
