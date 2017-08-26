#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joysticks.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "rpyt_based_velocity_controller_config.pb.h"

/**
 * @brief A controller that passes joystick commands to a drone's RPYT
 * controller
 */
class ManualVelocityController
: public Controller<std::tuple<Joysticks, VelocityYaw>, EmptyGoal, RollPitchYawThrust> {
public:
  /**
  * 
  */
  ManualVelocityController(RPYTBasedVelocityControllerConfig &rpyt_vel_ctlr_config)
  : rpyt_vel_ctlr_config_(rpyt_vel_ctlr_config),
  rpyt_vel_ctlr(rpyt_vel_ctlr_config_){}

  /**
   * @brief Destructor
   */
  virtual ~ManualVelocityController() {}

protected:
  /**
   * @brief Run the control loop.  Converts Joystick commands to Velocity.
   * @param sensor_data Joystick commands to be converted into Velocity.
   * @param goal Goal is not used here
   * @param control Velocity to send to hardware
   * return True if successfully converted sensor data to control
   */
  virtual bool runImplementation(std::tuple<Joysticks, VelocityYaw> sensor_data,
    EmptyGoal goal,
    RollPitchYawThrust &control);
  /**
  * @brief Default implementation since there is no concept of convergence
  * for manual rpyt controller
  * @return True always
  */
  virtual ControllerStatus isConvergedImplementation(std::tuple<Joysticks, VelocityYaw> sensor_data, EmptyGoal goal) {
    return ControllerStatus::Completed;
  }

private:
    /**
  * @brief Generic map function to map input range to output range
  *
  * @param input Input value to map
  * @param input_min Min for input
  * @param input_max Max for input
  * @param output_min Min for output
  * @param output_max Max for output
  *
  * @return Map the input based on input range to output in output range
  */
  double map(double input, double input_min, double input_max,
   double output_min, double output_max);
  /**
  *
  */
  RPYTBasedVelocityControllerConfig &rpyt_vel_ctlr_config_;
  /**
  * @brief Internal controller to get rpyt from desired velocity
  */
  RPYTBasedVelocityController rpyt_vel_ctlr;
};
