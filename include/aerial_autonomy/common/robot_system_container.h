#pragma once

/**
* @brief Container class that stores robot system
* and provides it to only selected friend classes
*
* @tparam RobotSystemT robot system type to store
*/
template <class RobotSystemT> class RobotSystemContainer {
  // Add friend classes that can use the robot system
  template <class RobotSystemT1, class LogicStateMachineT>
  friend class InternalActionFunctor;

  template <class EventT, class RobotSystemT1, class LogicStateMachineT>
  friend class ActionFunctor;

  template <class EventT, class RobotSystemT1, class LogicStateMachineT>
  friend class GuardFunctor;

  template <class RobotSystemT1, class LogicStateMachineT>
  friend class EventAgnosticActionFunctor;

  template <class RobotSystemT1, class LogicStateMachineT>
  friend class EventAgnosticGuardFunctor;

protected:
  /**
  * @brief robot system i.e stored.
  */
  RobotSystemT &robot_system_;

  /**
  * @brief Convenience function to return robot system
  *
  * @return robot system stored
  */
  RobotSystemT &operator()() { return robot_system_; }

public:
  /**
  * @brief Constructor to store robot system
  *
  * @param robot_system external robot system that is stored
  */
  RobotSystemContainer(RobotSystemT &robot_system)
      : robot_system_(robot_system) {}
};
