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
  template <class FSM, class SourceState, class TargetState>
  void operator()(EventT const &event, FSM &logic_state_machine, SourceState &,
                  TargetState &) {
    static_assert(
        std::is_convertible<decltype(
                                logic_state_machine.robot_system_container_()),
                            RobotSystemT &>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    static_assert(
        std::is_base_of<FSM, LogicStateMachineT>::value,
        "Template Logic state machine arg is not subclass of provided FSM");
    LogicStateMachineT *logic_state_machine_cast =
        static_cast<LogicStateMachineT *>(&logic_state_machine);
    run(event, logic_state_machine.robot_system_container_(),
        *logic_state_machine_cast);
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
  template <class FSM, class SourceState, class TargetState>
  bool operator()(EventT const &event, FSM &logic_state_machine, SourceState &,
                  TargetState &) {
    static_assert(
        std::is_convertible<decltype(
                                logic_state_machine.robot_system_container_()),
                            RobotSystemT &>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    static_assert(
        std::is_base_of<FSM, LogicStateMachineT>::value,
        "Template Logic state machine arg is not subclass of provided FSM");
    LogicStateMachineT *logic_state_machine_cast =
        static_cast<LogicStateMachineT *>(&logic_state_machine);
    return guard(event, logic_state_machine.robot_system_container_(),
                 *logic_state_machine_cast);
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
  template <class EventT, class FSM, class SourceState, class TargetState>
  void operator()(EventT const &, FSM &logic_state_machine, SourceState &,
                  TargetState &) {
    static_assert(
        std::is_convertible<decltype(
                                logic_state_machine.robot_system_container_()),
                            RobotSystemT &>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    static_assert(
        std::is_base_of<FSM, LogicStateMachineT>::value,
        "Template Logic state machine arg is not subclass of provided FSM");
    LogicStateMachineT *logic_state_machine_cast =
        static_cast<LogicStateMachineT *>(&logic_state_machine);
    run(logic_state_machine.robot_system_container_(),
        *logic_state_machine_cast);
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
  template <class EventT, class FSM, class SourceState, class TargetState>
  bool operator()(EventT const &, FSM &logic_state_machine, SourceState &,
                  TargetState &) {
    static_assert(
        std::is_convertible<decltype(
                                logic_state_machine.robot_system_container_()),
                            RobotSystemT &>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    static_assert(
        std::is_base_of<FSM, LogicStateMachineT>::value,
        "Template Logic state machine arg is not subclass of provided FSM");
    LogicStateMachineT *logic_state_machine_cast =
        static_cast<LogicStateMachineT *>(&logic_state_machine);
    return guard(logic_state_machine.robot_system_container_(),
                 *logic_state_machine_cast);
  }

  /**
  * @brief virtual destructor to get polymorphism
  */
  virtual ~EventAgnosticGuardFunctor() {}
};

/**
* @brief Internal action that returns whether it processed an event
*
* This action functor performs logic checks and processes events
* on the logic state machine. It also returns true if it processed
* events or not. This is used with ShortingActionSequence to chain
* internal actions and stop executing actions once an action processes
* events with logic state machine
*
* @tparam RobotSystemT The robot system used in the guard
* @tparam LogicStateMachineT The logic state machine used to retrieve robot
* system
*/
template <class RobotSystemT, class LogicStateMachineT>
struct InternalActionFunctor {
  /**
    * @brief Override this run function for different sub classes.
    * This function performs the logic checking for each state and
    * processes events on the state machine. It returns true if
    * an action is processed/ false otherwise.
    *
    * @param robot_system Provides sensor data and allows for controlling
    * hardware
    * @param logic_state_machine Backend of logic State Machine. can send events
    * using this.
    */
  virtual bool action(RobotSystemT &robot_system,
                      LogicStateMachineT &logic_state_machine) = 0;

  /**
   * @brief operator () Internally calls run function
   * @param logic_state_machine Backend of logic State Machine. can send
   * events using this.
   */
  template <class EventT, class FSM, class SourceState, class TargetState>
  bool operator()(EventT const &, FSM &logic_state_machine, SourceState &,
                  TargetState &) {
    static_assert(
        std::is_convertible<decltype(
                                logic_state_machine.robot_system_container_()),
                            RobotSystemT &>::value,
        "Robot system in logic state machine is not the same as one used in "
        "action functor");
    static_assert(
        std::is_base_of<FSM, LogicStateMachineT>::value,
        "Template Logic state machine arg is not subclass of provided FSM");
    LogicStateMachineT *logic_state_machine_cast =
        static_cast<LogicStateMachineT *>(&logic_state_machine);
    return action(logic_state_machine.robot_system_container_(),
                  *logic_state_machine_cast);
  }

  /**
  * @brief virtual destructor to get polymorphism
  */
  virtual ~InternalActionFunctor() {}
};

/**
 * @brief The InternalTransitionEvent struct
 * used to trigger action behaviors in states
 */
struct InternalTransitionEvent {};
