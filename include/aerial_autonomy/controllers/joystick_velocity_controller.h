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
      Atomic<RPYTBasedVelocityControllerConfig>
          &rpyt_velocity_controller_config,
      JoystickVelocityControllerConfig &joystick_velocity_controller_config,
      double controller_timer_duration)
      : controller_timer_duration_(controller_timer_duration),
        rpyt_velocity_controller_config_(rpyt_velocity_controller_config),
        rpyt_velocity_controller_(rpyt_velocity_controller_config_,
                                  controller_timer_duration_),
        joystick_velocity_controller_config_(
            joystick_velocity_controller_config) {
    DATA_HEADER("joystick_velocity_controller") << "Channel1"
                                                << "Channel2"
                                                << "Channel3"
                                                << "Channel4"
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
                            EmptyGoal goal);

private:
  /**
  * @brief Controller timestep
  */
  double controller_timer_duration_;
  /**
  * @ Config for rpyt velocity controller
  */
  Atomic<RPYTBasedVelocityControllerConfig> &rpyt_velocity_controller_config_;
  /**
  * @brief Internal controller to get rpyt from desired velocity
  */
  RPYTBasedVelocityController rpyt_velocity_controller_;
  /**
  * @brief Controller config for manual velocity controller
  */
  JoystickVelocityControllerConfig joystick_velocity_controller_config_;
};
