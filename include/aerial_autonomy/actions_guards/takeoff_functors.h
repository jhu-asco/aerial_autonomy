#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/types/completed_event.h>
#include <parsernode/common.h>

using namespace basic_events;

template <class LogicStateMachineT>
struct TakeoffTransitionActionFunctor_
    : ActionFunctor<Takeoff, UAVSystem, LogicStateMachineT> {
  void run(const Takeoff &, UAVSystem &robot_system, LogicStateMachineT &) {
    robot_system.takeOff();
  }
};

template <class LogicStateMachineT>
struct TakeoffAbortActionFunctor_
    : ActionFunctor<Abort, UAVSystem, LogicStateMachineT> {
  void run(const Abort &, UAVSystem &robot_system, LogicStateMachineT &) {
    robot_system.land();
  }
};

template <class LogicStateMachineT>
struct TakeoffTransitionGuardFunctor_
    : GuardFunctor<Takeoff, UAVSystem, LogicStateMachineT> {
  bool guard(const Takeoff &, UAVSystem &robot_system, LogicStateMachineT &) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    bool result = false;
    // Check for voltage
    // TODO Set a parameter for minimum battery percent
    if (data.batterypercent > 40) {
      result = true;
    } else {
      std::cout << "Battery too low! " << data.batterypercent
                << "% Cannot takeoff" << std::endl;
    }
    return result;
  }
};

template <class LogicStateMachineT>
struct TakeoffInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(const InternalTransitionEvent &, UAVSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // If battery too low abort and goto landed mode
    if (data.batterypercent < 40) {
      logic_state_machine.process_event(Land());
    }
    // Transition to hovering state once reached high altitude
    // TODO add a parameter for transitioning from takeoff to hovering
    if (data.altitude > 1.0) {
      logic_state_machine.process_event(Completed());
    }
  }
};

template <class LogicStateMachineT>
using TakingOff_ = BaseState<UAVSystem, LogicStateMachineT,
                             TakeoffInternalActionFunctor_<LogicStateMachineT>>;
