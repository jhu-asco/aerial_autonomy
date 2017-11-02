#pragma once

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Type index
#include <typeindex>

// Robot system container
#include <aerial_autonomy/common/robot_system_container.h>

// state machine config
#include "base_state_machine_config.pb.h"

#include <aerial_autonomy/common/unordered_heterogeneous_map.h>

/**
* @brief Base state machine
*/
template <class RobotSystemT> class BaseStateMachine {

protected:
  /**
  * @brief type index to store the event that did not trigger any transition
  */
  std::type_index no_transition_event_index_ = typeid(NULL);

  /**
  * @brief Maps states to configurations
  */
  UnorderedHeterogeneousMap<std::type_index> config_map_;

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

  const BaseStateMachineConfig &base_state_machine_config_;

  /**
  * @brief Returns the index of the event that did not trigger any transition
  * @return The no-transition event index
  */
  std::type_index get_no_transition_event_index() const {
    return no_transition_event_index_;
  }

  /**
   * @brief Store robot system and state machine config. robot system
   * is accessible to actions, guards. state machine config is
   * read only
   *
   * @param robot_system  Provides robot system to actions, guards
   * @param base_state_machine_config Config for statemachine. Other
   * statemachines will add their config as subclass inside proto.
   */
  BaseStateMachine(RobotSystemT &robot_system,
                   const BaseStateMachineConfig &base_state_machine_config)
      : robot_system_container_(robot_system),
        base_state_machine_config_(base_state_machine_config) {}

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

  /**
  * @brief Getter for state configuration map
  * @return The config map
  */
  const UnorderedHeterogeneousMap<std::type_index> &configMap() {
    return config_map_;
  }
};
