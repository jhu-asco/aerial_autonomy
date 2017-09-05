#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"

/**
 * @brief A controller that passes joystick commands to a drone's RPYT
 * controller
 */
class ManualRPYTController
    : public Controller<JoystickYaw, EmptyGoal, RollPitchYawThrust> {
public:
  /**
   * @brief Destructor
   */
  virtual ~ManualRPYTController() {}

protected:
  /**
   * @brief Run the control loop.  Converts Joystick commands to RPYT.
   * @param sensor_data Joystick commands to be converted into RPYT.
   * @param goal Goal is not used here
   * @param control RPYT to send to hardware
   * return True if successfully converted sensor data to control
   */
  virtual bool runImplementation(JoystickYaw sensor_data, EmptyGoal goal,
                                 RollPitchYawThrust &control);
  /**
  * @brief Default implementation since there is no concept of convergence
  * for manual rpyt controller
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(JoystickYaw, EmptyGoal) {
    return ControllerStatus(ControllerStatus::Completed);
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
};
