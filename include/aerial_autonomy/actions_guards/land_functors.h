#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/quadrotor_system.h>
#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/types/completed_event.h>
#include <parsernode/common.h>

using namespace basic_events;

template <class LogicStateMachineT>
struct LandTransitionActionFunctor_
    : ActionFunctor<Land, QuadRotorSystem, LogicStateMachineT> {
  void run(const Land &, QuadRotorSystem &robot_system, LogicStateMachineT &) {
    robot_system.land();
  }
};

// TODO (Gowtham) How to abort Land??

template <class LogicStateMachineT>
struct LandInternalActionFunctor_
    : InternalActionFunctor<QuadRotorSystem, LogicStateMachineT> {
  void run(const InternalTransitionEvent &, QuadRotorSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getQuadData();
    // Can also use quad status here TODO (Gowtham)
    if (data.altitude < 0.1) {
      logic_state_machine.process_event(Completed());
    }
  }
};

template <class LogicStateMachineT>
using Landing_ = BaseState<QuadRotorSystem, LogicStateMachineT,
                           LandInternalActionFunctor_<LogicStateMachineT>>;
