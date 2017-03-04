#pragma once

// Static asserts
#include <type_traits>

/**
* @brief This class provides run function for transitions and internal actions.
* Derived states have to implement the specific run function
* behavior using this base class
*
* @tparam EventT Event triggering the action
* @tparam RobotSystemT robot system used to get sensor data/perform actions
* @tparam LogicStateMachineT logic state machine to trigger events
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
   * @param event Event triggering the action
   */
  virtual void run(EventT const &event, RobotSystemT &robot_system,
                   LogicStateMachineT &logic_state_machine) = 0;

  /**
     * @brief operator () Internally calls run function
     * @param logic_state_machine Backend of logic State Machine. can send
     * events using this.
     * @param event Event triggering the action
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

  /**
  * @brief Virtual destructor to obtain polymorphism
  */
  virtual ~ActionFunctor() {}
};
/**
* @brief This class provides guard function for transitions and internal actions.
* Derived states have to implement the specific guard function
* behavior using this base class. The guard function checks for appropriate
* conditions to ensure actions can be performed
*
* @tparam EventT Event triggering the action
* @tparam RobotSystemT robot system used to get sensor data/perform actions
* @tparam LogicStateMachineT logic state machine to trigger events
*/
template <class EventT, class RobotSystemT, class LogicStateMachineT>
struct GuardFunctor {
  /**
   * @brief Override this guard function for different sub classes.
   * This function decides whether to call the run function for each state
   * @param robot_system Provides sensor data and allows for controlling
   * hardware
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   * @param event Event triggering the action
   */
  virtual bool guard(EventT const &event, RobotSystemT &robot_system,
                     LogicStateMachineT &logic_state_machine) = 0;
  /**
   * @brief operator () Internally calls guard function
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   * @param event Event triggering the action
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

/**
* @brief Internal action that implements the state transition logic
*
* The state transition logic triggers actions based on
* sensor data obtained from robot system
*
* @tparam RobotSystemT  robot system type that is used in internal action
* @tparam LogicStateMachineT Logic state machine backend type to trigger events
*/
template <class RobotSystemT, class LogicStateMachineT>
using InternalActionFunctor =
    ActionFunctor<InternalTransitionEvent, RobotSystemT, LogicStateMachineT>;
