#pragma once

#include <boost/thread/mutex.hpp>

/**
* @brief Status of the controller
*/
enum class ControllerStatus {
  Active,    ///< Controller active
  Completed, ///< Controller completed
  Critical   ///< Controller is critical and unable to continue
};

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
   * @return Control values to send to hardware
   */
  virtual ControlType run(SensorDataType sensor_data,
                          ControllerStatus &status) {
    GoalType goal;
    {
      boost::mutex::scoped_lock lock(goal_mutex_);
      goal = goal_;
    }
    return runImplementation(sensor_data, goal, status);
  }

  /**
  * @brief Check if controller is converged
  *
  * @param sensor_data Sensor data to compare against goal for convergence
  * checking
  *
  * @return true if converged/false otherwise
  */
  bool isConverged(SensorDataType sensor_data) {
    GoalType goal;
    {
      boost::mutex::scoped_lock lock(goal_mutex_);
      goal = goal_;
    }
    return isConvergedImplementation(sensor_data, goal);
  }
  /**
   * @brief set the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   * @param goal The goal for control loop
   */
  virtual void setGoal(GoalType goal) {
    boost::mutex::scoped_lock lock(goal_mutex_);
    goal_ = goal;
  }
  /**
   * @brief get the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   */
  virtual GoalType getGoal() const {
    boost::mutex::scoped_lock lock(goal_mutex_);
    return goal_;
  }
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
   * @param status Controller status
   * @return Control values to send to hardware
   */
  virtual ControlType runImplementation(SensorDataType sensor_data,
                                        GoalType goal,
                                        ControllerStatus &status) = 0;
  /**
  * @brief Implementation for checking convergence to be implemented by
  * subclasses. This function is called after runImplementation function
  * and that can be used to store any information required for checking
  * convergence.
  *
  * @param sensor_data Data to be used for checking convergence
  * @param goal This is compared against sensor data
  *
  * @return true if converged/false otherwise
  */
  virtual bool isConvergedImplementation(SensorDataType sensor_data,
                                         GoalType goal) = 0;

private:
  /**
  * @brief Synchronize set goal and running controller implementation.
  */
  mutable boost::mutex goal_mutex_;
  /**
  * @brief store goal internally for usage with Controller::runImplementation
  */
  GoalType goal_;
};
