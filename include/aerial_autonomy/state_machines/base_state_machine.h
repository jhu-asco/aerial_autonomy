#pragma once

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Type index
#include <typeindex>

// Robot system container
#include <aerial_autonomy/common/robot_system_container.h>

/**
* @brief Base state machine
*/
template <class RobotSystemT> class BaseStateMachine {

protected:
  /**
  * @brief type index to store the event that did not trigger any transition
  */
  std::type_index no_transition_event_index_ = typeid(NULL);

public:
  /**
  * @brief robot system container used by states to get sensor data and send
  * commands
  * Use the operator function robot_system_container_() to retrieve robot system
  * in Action, Guard Functors. Other classes cannot access the robot system
  *
  * Decision Logic:
  * The robot system container ensures that robot system is available
  * to only few friend classes such as action functor, guard functors and
  * is not available to others. This is necessary since we do not expect
  * the logic state machine to contain the robot system. This was the only
  * reasonable way found to share robot system across action and guard functors.
  */
  RobotSystemContainer<RobotSystemT> robot_system_container_;

  /**
  * @brief Returns the index of the event that did not trigger any transition
  * @return The no-transition event index
  */
  std::type_index get_no_transition_event_index() const {
    return no_transition_event_index_;
  }

  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_system robot system that is stored internally
  * and shared with events
  */
  BaseStateMachine(RobotSystemT &robot_system)
      : robot_system_container_(robot_system) {}

  /**
  * @brief Print event typeid if no action present for the corresponding event
  *
  * @tparam FSM Backend to trigger events etc
  * @tparam Event Event type that triggered no transition
  * @param e event instance
  * @param  state_index The index of the state where the no transition event is
  * received
  */
  template <class FSM, class Event>
  void no_transition(Event const &e, FSM &, int state_index) {
    no_transition_event_index_ = typeid(e);
    LOG(WARNING) << "Event " << no_transition_event_index_.name()
                 << " triggered no transition";
  }
};
