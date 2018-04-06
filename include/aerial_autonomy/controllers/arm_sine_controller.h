#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/empty_sensor.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "arm_sine_controller_config.pb.h"
#include <chrono>

/**
* @brief Typedef for controller output
*/
using JointAngles = std::vector<double>;

/**
 * @brief A controller that commands joint angles in a sinusoid
 * manner for system ID
 */
class ArmSineController
    : public Controller<EmptySensor, EmptyGoal, JointAngles> {
public:
  /**
  * @brief Constructor
  */
  ArmSineController(ArmSineControllerConfig config);
  /**
   * @brief Destructor
   */
  virtual ~ArmSineController() {}

  /**
  * @brief Set t0 to current time
  */
  void setZeroTime();

  /**
  * @brief Time elapsed since zero time (t0)
  *
  * @return time in seconds
  */
  std::chrono::duration<double> duration();

protected:
  /**
   * @brief Move arm in sinusoid manner
   * @param sensor_data Joystick commands to be converted into RPYT.
   * @param goal Goal is not used here
   * @param control RPYT to send to hardware
   * return True if successfully converted sensor data to control
   */
  virtual bool runImplementation(EmptySensor sensor_data, EmptyGoal goal,
                                 JointAngles &control);
  /**
  * @brief Default implementation since there is no concept of convergence
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(EmptySensor, EmptyGoal) {
    return ControllerStatus(ControllerStatus::Completed);
  }

protected:
  /**
   * @brief Zero time to substract from current time
   */
  std::chrono::time_point<std::chrono::high_resolution_clock> t0_;
  /**
  * @brief Sine properties for each joint
  */
  ArmSineControllerConfig config_;
};
