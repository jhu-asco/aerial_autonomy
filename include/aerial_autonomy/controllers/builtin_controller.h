#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_sensor.h"

/**
 * @brief A controller that simply outputs the set goal
 */
template <class GoalType>
class BuiltInController : public Controller<EmptySensor, GoalType, GoalType> {
public:
  /**
   * @brief Destructor
   */
  virtual ~BuiltInController() {}

protected:
  /**
   * @brief Run the control loop.  Simply returns the goal.
   * @param sensor_data Empty sensor data struct since no sensing is required.
   * @param goal Goal set-point
   * @return Goal to send to hardware
   */
  virtual GoalType runImplementation(EmptySensor sensor_data, GoalType goal) {
    return goal;
  }
};
