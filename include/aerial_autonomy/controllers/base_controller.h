#pragma once

#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/common/controller_status.h"

/**
* @brief Base Controller class
*
* subclass should implement the runImplementation function which
* takes as input the sensor data and the desired goal and returns
* a control value.
*
* @tparam SensorDataType The type of sensor the controller takes in
* @tparam GoalType  The type of goal the controller takes in
* @tparam ControlType The type of control the controller returns
*/
template <class SensorDataType, class GoalType, class ControlType>
class Controller {
public:
  /**
   * @brief Run the control loop and return control arguments
   * @param sensor_data Data required for control loop. Can also be
   * estimator data
   * @param control Control values to send to hardware
   * @return True if the control run is successful
   */
  virtual bool run(SensorDataType sensor_data, ControlType &control) {
    return runImplementation(sensor_data, goal_, control);
  }

  /**
  * @brief Check if controller is converged
  *
  * @param sensor_data Sensor data to compare against goal for convergence
  * checking
  *
  * @return status that contains different states the controller and debug info.
  */
  ControllerStatus isConverged(SensorDataType sensor_data) {
    return isConvergedImplementation(sensor_data, goal_);
  }
  /**
   * @brief set the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   * @param goal The goal for control loop
   */
  virtual void setGoal(GoalType goal) { goal_ = goal; }
  /**
   * @brief get the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   */
  virtual GoalType getGoal() const { return goal_; }
  /**
   * @brief Destructor
   */
  virtual ~Controller() {}

protected:
  /**
   * @brief Run the control loop and return control arguments
   * @param sensor_data Data required for control loop. Can also be
   * estimator data
   * @param goal The set-point for the controller
   * @param control Output Control values to send to hardware
   * @return True if the run is successful
   */
  virtual bool runImplementation(SensorDataType sensor_data, GoalType goal,
                                 ControlType &control) = 0;
  /**
  * @brief Implementation for checking convergence to be implemented by
  * subclasses. This function is called after runImplementation function
  * and that can be used to store any information required for checking
  * convergence.
  *
  * @param sensor_data Data to be used for checking convergence
  * @param goal This is compared against sensor data
  *
  * @return status that contains different states the controller and debug info.
  */
  virtual ControllerStatus isConvergedImplementation(SensorDataType sensor_data,
                                                     GoalType goal) = 0;

private:
  /**
  * @brief store goal internally for usage with Controller::runImplementation
  */
  Atomic<GoalType> goal_;
};
