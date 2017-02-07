#pragma once
/** This class provides internal run function for different states.
 * Derived states have to implement the specific run function
 * behavior using this base class
 */
template <class RobotSystemT, class LogicStateMachineT> struct BaseRunFunctor {
  /**
   * @brief Override this run function for different sub classes.
   * This function performs the logic checking for each state
   * @param robot_system Provides sensor data and allows for controlling
   * hardware
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   */
  virtual void run(RobotSystemT const &robot_system,
                   LogicStateMachineT &logic_state_machine) = 0;

  /**
     * @brief operator () Internally calls run function
     * @param robot_system Provides sensor data and allows for controlling
     * hardware
     * @param logic_state_machine Backend of logic State Machine. can send
     * events using this.
     */
  template <class SourceState, class TargetState>
  void operator()(RobotSystemT const &robot_system,
                  LogicStateMachineT &logic_state_machine, SourceState &,
                  TargetState &) {
    run(robot_system, logic_state_machine);
  }

  virtual ~BaseRunFunctor() {}
};

template <class RobotSystemT, class LogicStateMachineT>
struct BaseGuardFunctor {
  /**
   * @brief Override this guard function for different sub classes.
   * This function decides whether to call the run function for each state
   * @param robot_system Provides sensor data and allows for controlling
   * hardware
   * @param logic_state_machine Backend of logic State Machine. can send events
   * using this.
   */
  virtual bool guard(RobotSystemT const &robot_system,
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
  bool operator()(RobotSystemT const &robot_system,
                  LogicStateMachineT &logic_state_machine, SourceState &,
                  TargetState &) {
    return guard(robot_system, logic_state_machine);
  }
  /**
     * @brief Destructor
     */
  virtual ~BaseGuardFunctor() {}
};
