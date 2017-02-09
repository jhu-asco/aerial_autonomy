#pragma once

#include <boost/thread/mutex.hpp>

template <class SensorDataType, class GoalType, class ControlType>
class Controller {
public:
  /**
   * @brief Run the control loop and return control arguments
   * @param sensor_data Data required for control loop. Can also be
   * estimator data
   * @return Control values to send to hardware
   */
  virtual ControlType run(SensorDataType sensor_data) {
    GoalType goal;
    {
      boost::mutex::scoped_lock lock(goal_mutex);
      goal = goal_;
    }
    return runImpl(sensor_data, goal);
  }
  /**
   * @brief set the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   * @param goal The goal for control loop
   */
  virtual void setGoal(GoalType goal) {
    boost::mutex::scoped_lock lock(goal_mutex);
    goal_ = goal;
  }
  /**
   * @brief set the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   * @param goal The goal for control loop
   */
  virtual GoalType getGoal() {
    boost::mutex::scoped_lock lock(goal_mutex);
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
   * @return Control values to send to hardware
   */
  virtual ControlType runImpl(SensorDataType sensor_data, GoalType goal) = 0;

private:
  boost::mutex goal_mutex;
  GoalType goal_;
};
