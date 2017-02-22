#pragma once

// Static asserts
#include <type_traits>

/** This class provides internal run function for different states.
 * Derived states have to implement the specific run function
 * behavior using this base class
 */
template <class EventT, class RobotSystemT, class LogicStateMachineT>
struct ActionFunctor {
  /**
   * @brief Override this run function for different sub classes.
   * This function performs the logic checking for each state
   * @param robot_system Provides sensor data and allows for controlling
   * hardware
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   */
  virtual void run(EventT const &event, RobotSystemT &robot_system,
                   LogicStateMachineT &logic_state_machine) = 0;

  /**
     * @brief operator () Internally calls run function
     * @param robot_system Provides sensor data and allows for controlling
     * hardware
     * @param logic_state_machine Backend of logic State Machine. can send
     * events using this.
     */
  template <class SourceState, class TargetState>
  void operator()(EventT const &event, LogicStateMachineT &logic_state_machine,
                  SourceState &, TargetState &) {
    static_assert(
        std::is_same<RobotSystemT &,
                     decltype(logic_state_machine.robot_system_)>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    run(event, logic_state_machine.robot_system_, logic_state_machine);
  }

  virtual ~ActionFunctor() {}
};
////// Change Guard functor to be similar to action functor
/// Add the guard functor to be a friend to robot wrapper
/// Put static assert to make sure logicstatemachine is supplying the same robot
/// system
template <class EventT, class RobotSystemT, class LogicStateMachineT>
struct GuardFunctor {
  /**
   * @brief Override this guard function for different sub classes.
   * This function decides whether to call the run function for each state
   * @param robot_system Provides sensor data and allows for controlling
   * hardware
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   */
  virtual bool guard(EventT const &event, RobotSystemT &robot_system,
                     LogicStateMachineT &logic_state_machine) = 0;
  /**
   * @brief operator () Internally calls guard function
   * @param robot_system Provides sensor data and allows for controlling
   * hardware
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   * @return the result of checking guard for the state
   */
  template <class SourceState, class TargetState>
  bool operator()(EventT const &event, LogicStateMachineT &logic_state_machine,
                  SourceState &, TargetState &) {
    static_assert(
        std::is_same<RobotSystemT &,
                     decltype(logic_state_machine.robot_system_)>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    return guard(event, logic_state_machine.robot_system_, logic_state_machine);
  }
  /**
     * @brief Destructor
     */
  virtual ~GuardFunctor() {}
};

/**
 * @brief The InternalTransitionEvent struct
 * used to trigger action behaviors in states
 */
struct InternalTransitionEvent {};

template <class RobotSystemT, class LogicStateMachineT>
using InternalActionFunctor =
    ActionFunctor<InternalTransitionEvent, RobotSystemT, LogicStateMachineT>;
