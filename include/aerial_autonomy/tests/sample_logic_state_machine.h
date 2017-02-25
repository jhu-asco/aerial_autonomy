#pragma once
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <type_traits>

struct EmptyRobotSystem {};

template <class RobotSystemT> class SampleLogicStateMachine_ {
  template <class EventT, class RobotSystemT1, class LogicStateMachineT>
  friend class ActionFunctor;
  template <class EventT, class RobotSystemT1, class LogicStateMachineT>
  friend class GuardFunctor;
  template <class RobotSystemT1, class LogicStateMachineT>
  friend class EventAgnosticActionFunctor;
  template <class RobotSystemT1, class LogicStateMachineT>
  friend class EventAgnosticGuardFunctor;
  std::type_index type_index_event_ = typeid(NULL);
  RobotSystemT &robot_system_;

public:
  SampleLogicStateMachine_(RobotSystemT &robot_system)
      : robot_system_(robot_system) {}
  template <class Event> void process_event(const Event &event) {
    type_index_event_ = typeid(event);
  }
  std::type_index getProcessEventTypeId() { return type_index_event_; }
};

using SampleLogicStateMachine = SampleLogicStateMachine_<EmptyRobotSystem>;
using UAVLogicStateMachine = SampleLogicStateMachine_<UAVSystem>;
