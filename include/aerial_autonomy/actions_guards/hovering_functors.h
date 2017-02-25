#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/basic_events.h>
#include <parsernode/common.h>

using namespace basic_events;

template <class LogicStateMachineT>
struct HoveringInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(const InternalTransitionEvent &, UAVSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // Transition to hovering state once reached high altitude
    // Can also use uav status here TODO (Gowtham)
    if (data.batterypercent < 40) {
      logic_state_machine.process_event(Land());
    }
  }
};

template <class LogicStateMachineT>
using Hovering_ = BaseState<UAVSystem, LogicStateMachineT,
                            HoveringInternalActionFunctor_<LogicStateMachineT>>;
