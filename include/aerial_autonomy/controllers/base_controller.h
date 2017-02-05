#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

template <class SensorDataType, class GoalType, class ControlType>
class Controller {
public:
  /**
   * @brief Run the control loop and return control arguments
   * @param sensor_data Data required for control loop. Can also be
   * estimatordata
   * @return Control values to send to hardware
   */
  virtual ControlType run(SensorDataType sensor_data) = 0;
  // lock and set the desired goal for the control logic
  /**
   * @brief set the goal condition for the controller. Should use
   * internal locking as the run function can be called from a separate thread
   * @param goal The goal for control loop
   */
  virtual void setGoal(GoalType goal) = 0;
  /**
   * @brief Destructor
   */
  virtual ~Controller() {}
};
#endif // BASE_CONTROLLER_H
