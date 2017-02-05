#ifndef BASE_CONTROL_HARDWARE_CONNECTOR_H
#define BASE_CONTROL_HARDWARE_CONNECTOR_H
#include <aerial_autonomy/controllers/base_controller.h>

struct AbstractControllerHardwareConnector {
public:
  virtual void run() = 0;
  virtual ~AbstractControllerHardwareConnector() {}
};

template <class SensorDataType, class GoalType, class ControlType>
class ControllerHardwareConnector : public AbstractControllerHardwareConnector {
public:
  ControllerHardwareConnector(
      Controller<SensorDataType, GoalType, ControlType> &ctrlr)
      : AbstractControllerHardwareConnector(), ctrlr_(ctrlr) {}

  virtual SensorDataType extractSensorData() = 0;
  virtual void sendHardwareCommands(ControlType controls) = 0;

  virtual void run() {
    // Get latest sensor data
    // run the controller
    // send the data back to hardware manager
    SensorDataType sensor_data = extractSensorData();
    ControlType control = ctrlr_.run(sensor_data);
    sendHardwareCommands(control);
  }
  // Can be made into a macro
  void setGoal(GoalType goal) {
    // call the controller set goal function
    ctrlr_.setGoal(goal);
  }

private:
  Controller<SensorDataType, GoalType, ControlType> &ctrlr_;
};
#endif // BASE_CONTROL_HARDWARE_CONNECTOR_H
