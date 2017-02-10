#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joysticks_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"

/**
 * @brief A controller that passes joystick commands to a drone's RPYT
 * controller
 */
class ManualRPYTController
    : public Controller<JoysticksYaw, EmptyGoal, RollPitchYawThrust> {
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
   * @return RPYT to send to hardware
   */
  virtual RollPitchYawThrust runImplementation(JoysticksYaw sensor_data,
                                               EmptyGoal goal);

private:
  double map(double input, double input_min, double input_max,
             double output_min, double output_max);
};
