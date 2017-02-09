#pragma once
#include <aerial_autonomy/controllers/base_controller.h>

/**
* @brief Base for ControllerHardwareConnector class
*/
struct AbstractControllerHardwareConnector {
public:
  virtual void run() = 0;
  virtual ~AbstractControllerHardwareConnector() {}
};

/**
* @brief Performs a single step of extracting data, running controller
* and sending data back to hardware
*
* @tparam SensorDataType  Type of data to take from hardware
* @tparam GoalType        Type of goal for controller
* @tparam ControlType     Type of control sent to hardware
*/
template <class SensorDataType, class GoalType, class ControlType>
class ControllerHardwareConnector : public AbstractControllerHardwareConnector {
public:
  ControllerHardwareConnector(
      Controller<SensorDataType, GoalType, ControlType> &controller)
      : AbstractControllerHardwareConnector(), controller_(controller) {}

  /**
   * @brief Extracts sensor data, run controller and send data back to hardware
   */
  virtual void run() {
    // Get latest sensor data
    // run the controller
    // send the data back to hardware manager
    SensorDataType sensor_data = extractSensorData();
    ControlType control = controller_.run(sensor_data);
    sendHardwareCommands(control);
  }
  /**
   * @brief Set the goal for controller
   *
   * @param goal Goal for controller
   */
  void setGoal(GoalType goal) {
    controller_.setGoal(goal);
  }
  /**
   * @brief Get the goal for controller
   *
   * @return Goal for controller
   */
  GoalType getGoal() {
    return controller_.getGoal();
  }

protected:
  /**
   * @brief  extract relevant data from hardware/estimators
   *
   * @return data structure needed for controller to perform step function
   */
  virtual SensorDataType extractSensorData() = 0;

  /**
   * @brief  Send hardware commands for example quadrotor rpy
   *
   * @param controls Data structure the quadrotor is expecting
   */
  virtual void sendHardwareCommands(ControlType controls) = 0;

private:
  /**
   * @brief  controller class used to perform step function
   */
  Controller<SensorDataType, GoalType, ControlType> &controller_;
};
