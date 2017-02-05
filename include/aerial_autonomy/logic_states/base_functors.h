#ifndef BASE_FUNCTORS_H
#define BASE_FUNCTORS_H

/** This class provides internal run function for different states.
 * Derived states have to implement the specific run function
 * behavior using this base class
 */
template <class RobotSystemT, class LSMT> struct BaseRunFunctor {
  /**
   * @brief Override this run function for different sub classes.
   * This function performs the logic checking for each state
   * @param rs Provides sensor data and allows for controlling hardware
   * @param lsm Backend of logic State Machine. can send events using this.
   */
  virtual void run(RobotSystemT const &rs, LSMT &lsm) = 0;

  /**
     * @brief operator () Internally calls guard function
     * @param rs Provides sensor data and allows for controlling hardware
     * @param lsm Backend of logic State Machine. can send events using this.
     * @return output of guard function
     */
  template <class SourceState, class TargetState>
  void operator()(RobotSystemT const &rs, LSMT &lsm, SourceState &,
                  TargetState &) {
    run(rs, lsm);
  }

  virtual ~BaseRunFunctor() {}
};

template <class RobotSystemT, class LSMT> struct BaseGuardFunctor {
  /**
   * @brief Override this guard function for different sub classes.
   * This function decides whether to call the run function for each state
   * @param rs Provides sensor data and allows for controlling hardware
   * @param lsm Backend of logic State Machine. can send events using this.
   */
  virtual bool guard(RobotSystemT const &rs, LSMT &lsm) = 0;

  /**
   * @brief operator () Internally calls run function
   * @param rs Provides sensor data and allows for controlling hardware
   * @param lsm Backend of logic State Machine. can send events using this.
   */
  template <class SourceState, class TargetState>
  bool operator()(RobotSystemT const &rs, LSMT &lsm, SourceState &,
                  TargetState &) {
    return guard(rs, lsm);
  }
  /**
     * @brief Destructor
     */
  virtual ~BaseGuardFunctor() {}
};
#endif // BASE_FUNCTORS_H
