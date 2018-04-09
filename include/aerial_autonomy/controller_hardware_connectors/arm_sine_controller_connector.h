#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/arm_sine_controller.h"
#include <arm_parsers/arm_parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and moves the arm to a goal pose relative to the tracked target
 */
class ArmSineControllerConnector
    : public ControllerHardwareConnector<EmptySensor, EmptyGoal,
                                         std::vector<double>> {

  /**
  * @brief  typedef for parent class
  */
  using BaseConnector =
      ControllerHardwareConnector<EmptySensor, EmptyGoal, std::vector<double>>;

public:
  /**
   * @brief Constructor
   */
  ArmSineControllerConnector(ArmParser &arm_hardware,
                             ArmSineController &controller);

  /**
   * @brief Destructor
   */
  virtual ~ArmSineControllerConnector() {}
  /**
   * @brief Set the goal for controller
   *
   * @param goal Goal for controller
   */
  virtual void setGoal(EmptyGoal goal) {
    private_ref_controller_.setZeroTime();
    BaseConnector::setGoal(goal);
  }

protected:
  /**
  * @brief Extract sensor data
  *
  * @param Output sensor data
  *
  * @return True always
  */
  virtual bool extractSensorData(EmptySensor &);

  /**
   * @brief  Send joint angle commands to hardware
   *
   * @param angle commands to send to the arm
   */
  virtual void sendHardwareCommands(std::vector<double> controls);

private:
  /**
  * @brief Arm hardware to send commands
  */
  ArmParser &arm_hardware_;
  ArmSineController &private_ref_controller_;
};
