#pragma once

// Static asserts
#include <type_traits>

/**
* @brief Action Functor for a given event, robot system, state machine
*
* This class provides run function for state transitions
* Derived states have to implement the specific run function
* behavior using this base class.
*
* @tparam EventT  Event triggering this action
* @tparam RobotSystemT The robot system used in the action
* @tparam LogicStateMachineT The logic state machine used to trigger events
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
* @brief Action Functor for a given event, robot system, state machine
*
* This class provides guard function for state transitions
* Derived states have to implement the specific guard function
* behavior using this base class.
*
* @tparam EventT  Event triggering this action
* @tparam RobotSystemT The robot system used in the guard check
* @tparam LogicStateMachineT The logic state machine used to retrieve robot
* system
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
* @brief Action functor that does not require the event triggering it
*
* This action functor performs the same action irrespective of the event
* triggering it. For example takeoff, land actions
*
* @tparam RobotSystemT The robot system used in the action
* @tparam LogicStateMachineT The logic state machine used to trigger events
*/
template <class RobotSystemT, class LogicStateMachineT>
struct EventAgnosticActionFunctor {
  /**
    * @brief Override this run function for different sub classes.
    * This function performs the logic checking for each state
    * @param robot_system Provides sensor data and allows for controlling
    * hardware
    * @param logic_state_machine Backend of logic State Machine. can send events
    * using this.
    */
  virtual void run(RobotSystemT &robot_system,
                   LogicStateMachineT &logic_state_machine) = 0;

  /**
     * @brief operator () Internally calls run function
     * @param logic_state_machine Backend of logic State Machine. can send
     * events using this.
     */
  template <class EventT, class SourceState, class TargetState>
  void operator()(EventT const &, LogicStateMachineT &logic_state_machine,
                  SourceState &, TargetState &) {
    static_assert(
        std::is_same<RobotSystemT &,
                     decltype(logic_state_machine.robot_system_)>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    run(logic_state_machine.robot_system_, logic_state_machine);
  }

  /**
  * @brief virtual destructor to get polymorphism
  */
  virtual ~EventAgnosticActionFunctor() {}
};

/**
* @brief Guard functor that does not require the event triggering it
*
* This action functor performs the same guard check irrespective of the
* event triggering it.
*
* @tparam RobotSystemT The robot system used in the guard
* @tparam LogicStateMachineT The logic state machine used to retrieve robot
* system
*/
template <class RobotSystemT, class LogicStateMachineT>
struct EventAgnosticGuardFunctor {
  /**
    * @brief Override this run function for different sub classes.
    * This function performs the logic checking for each state
    * @param robot_system Provides sensor data and allows for controlling
    * hardware
    * @param logic_state_machine Backend of logic State Machine. can send events
    * using this.
    */
  virtual bool guard(RobotSystemT &robot_system,
                     LogicStateMachineT &logic_state_machine) = 0;

  /**
     * @brief operator () Internally calls run function
     * @param logic_state_machine Backend of logic State Machine. can send
     * events using this.
     */
  template <class EventT, class SourceState, class TargetState>
  bool operator()(EventT const &, LogicStateMachineT &logic_state_machine,
                  SourceState &, TargetState &) {
    static_assert(
        std::is_same<RobotSystemT &,
                     decltype(logic_state_machine.robot_system_)>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    return guard(logic_state_machine.robot_system_, logic_state_machine);
  }

  /**
  * @brief virtual destructor to get polymorphism
  */
  virtual ~EventAgnosticGuardFunctor() {}
};

/**
 * @brief The InternalTransitionEvent struct
 * used to trigger action behaviors in states
 */
struct InternalTransitionEvent {};
