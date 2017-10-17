#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "manual_rpyt_controller_config.pb.h"

/**
 * @brief A controller that passes joystick commands to a drone's RPYT
 * controller
 */
class ManualRPYTController
    : public Controller<JoystickYaw, EmptyGoal, RollPitchYawThrust> {
public:
  /**
  * @brief Default constructor
  */
  ManualRPYTController()
      : config_(ManualRPYTControllerConfig()),
        controller_timer_duration_(0.02) {}
  /**
  * @brief Explicit constructor
  */
  ManualRPYTController(ManualRPYTControllerConfig config,
                       double controller_timer_duration)
      : config_(config), controller_timer_duration_(controller_timer_duration) {
  }
  /**
  * @brief get last commanded yaw
  */
  double getLastCommandedYaw() { return last_yaw; }
  /**
  * @brief set last commanded yaw from sensor data
  */
  void setLastCommandedYaw(double last_commanded_yaw) {
    last_yaw = last_commanded_yaw;
  }
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
  * @brief Config for manual rpyt controller
  */
  ManualRPYTControllerConfig config_;
  /**
  * @brief controller timestep
  */
  double controller_timer_duration_;
  /**
  * @brief Last commanded yaw
  */
  double last_yaw = 0;
};
