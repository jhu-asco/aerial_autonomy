#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"

/**
 * @brief A controller that passes joystick commands to a drone's RPYT
 * controller
 */
class ManualRPYTController
    : public Controller<Joystick, EmptyGoal, RollPitchYawRateThrust> {
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
  virtual bool runImplementation(Joystick sensor_data, EmptyGoal goal,
                                 RollPitchYawRateThrust &control);
  /**
  * @brief Default implementation since there is no concept of convergence
  * for manual rpyt controller
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(Joystick, EmptyGoal) {
    return ControllerStatus(ControllerStatus::Completed);
  }
};
