#pragma once
#include <aerial_autonomy/common/atomic.h>
#include <aerial_autonomy/common/controller_status.h>
#include <aerial_autonomy/controllers/base_controller.h>
#include <aerial_autonomy/types/controller_groups.h>
#include <glog/logging.h>

/**
* @brief Base for ControllerConnector class
*/
struct AbstractControllerConnector {
public:
  /**
  * @brief Abstract run function.
  *
  * The run function should run controller
  * and send commands to hardware
  */
  virtual void run() = 0;

  /**
  * @brief Provide the status of the controller
  *
  * @return The status of the controller
  */
  virtual ControllerStatus getStatus() const = 0;

  /**
  * @brief Adds another connector to be dependent on the current connector. If
  * the current connector is active, the dependent connector also gets active.
  * It is assumed that the current connector will configure the dependent
  * connector so that activation is possible.
  *
  * The user should be CAREFUL to avoid any loops in dependent connectors
  *
  * @return the pointer to dependent connector
  */
  virtual AbstractControllerConnector *getDependentConnector() {
    return nullptr;
  }

  /**
  * @brief Return the type of hardware (ControllerGroup) used by the controller
  *
  * @return return the type of hardware used by the controller
  */
  virtual ControllerGroup getControllerGroup() const = 0;

  /**
  * @brief Destructor to get polymorphism
  */
  virtual ~AbstractControllerConnector() {}
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
class ControllerConnector : public AbstractControllerConnector {
public:
  /**
  * @brief ControlConnector runs the controller and sends the commands
  * to the receiver. The receiver can be another hardware/controller connector
  *
  * The subclass controlhardwareconnectors should take the required hardware in
  * the constructor
  * and implement the run function to get the command from controller and send
  * to hardware
  *
  * @param controller Controller to run during run function
  * @param controller_group Group the controller belongs to. Used to group the
  * connectors and ensure only one connector is running per group type
  */
  ControllerConnector(
      Controller<SensorDataType, GoalType, ControlType> &controller,
      ControllerGroup controller_group)
      : AbstractControllerConnector(), controller_group_(controller_group),
        controller_(controller) {}

  /**
   * @brief Extracts sensor data, run controller and send data back to hardware
   */
  virtual void run() {
    // Get latest sensor data
    // run the controller
    // send the data back to hardware manager
    SensorDataType sensor_data;
    ControlType control;
    if (!extractSensorData(sensor_data)) {
      status_ = ControllerStatus(ControllerStatus::Critical,
                                 "Cannot extract sensor data");
      return;
    }
    if (!controller_.run(sensor_data, control)) {
      status_ =
          ControllerStatus(ControllerStatus::Critical, "Cannot run controller");
      return;
    }
    sendControllerCommands(control);
    status_ = controller_.isConverged(sensor_data);
  }
  /**
   * @brief Set the goal for controller
   *
   * @param goal Goal for controller
   */
  virtual void setGoal(GoalType goal) {
    status_ = ControllerStatus(ControllerStatus::Active);
    controller_.setGoal(goal);
  }
  /**
   * @brief Get the goal for controller
   *
   * @return Goal for controller
   */
  GoalType getGoal() const { return controller_.getGoal(); }

  /**
  * @brief Return the type of hardware (ControllerGroup) used by the controller
  *
  * @return return the type of hardware used by the controller
  */
  ControllerGroup getControllerGroup() const { return controller_group_; }

  /**
  * @brief Provide the status of the controller
  *
  * @return The status of the controller
  */
  ControllerStatus getStatus() const { return status_; }

protected:
  /**
   * @brief  extract relevant data from hardware/estimators
   *
   * @param sensor_data Data to be updated based on sensor measurements
   *
   * @return true if extraction succeeded otherwise false
   */
  virtual bool extractSensorData(SensorDataType &sensor_data) = 0;

  /**
   * @brief  Send controller commands for example UAV rpy
   *
   * @param controls Data structure the UAV is expecting
   */
  virtual void sendControllerCommands(ControlType controls) = 0;

  /**
  * @brief Type of hardware controlled by the controller
  *
  * Used to group controllers. Only one controller will be running
  * per hardware.
  *
  */
  ControllerGroup controller_group_;

private:
  /**
   * @brief  controller class used to perform step function
   */
  Controller<SensorDataType, GoalType, ControlType> &controller_;
  /**
  * @brief Status of the controller
  */
  Atomic<ControllerStatus> status_;
};
