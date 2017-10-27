#pragma once
/**
 * @brief State which keeps track of how long it has been active since entry
 *
 * This class provides a wrapper for any action function and robot
 * system
 */

#include "aerial_autonomy/logic_states/base_state.h"
#include <chrono>

template <class RobotSystemT, class LogicStateMachineT, class ActionFctr>
class TimedState
    : public BaseState<RobotSystemT, LogicStateMachineT, ActionFctr> {
public:
  /**
  * @brief Function called when entering the state.  Logs time of entry.
  * @param e Event which triggered entry
  * @param fsm State's state machine
  * @tparam Event Type of event which triggered entry
  * @tparam FSM State machine type
  */
  template <class Event, class FSM> void on_entry(Event const &e, FSM &fsm) {
    // log start time
    entry_time_ = std::chrono::high_resolution_clock::now();
  }

  /**
  * @brief Get the amount of time spent in the state
  * @return The amount of time spent in the state
  */
  std::chrono::duration<double> timeInState() {
    return std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - entry_time_);
  }

  /**
   * @brief Destructor
   */
  virtual ~TimedState() {}

protected:
  std::chrono::time_point<std::chrono::high_resolution_clock> entry_time_;
};
