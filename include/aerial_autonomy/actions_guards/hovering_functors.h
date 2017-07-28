#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/manual_control_event.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <glog/logging.h>
#include <parsernode/common.h>

namespace be = uav_basic_events;

/**
* @brief Internal action when hovering.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct HoveringInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Checks for enough battery voltage and land if battery critical
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // If hardware is not allowing us to control UAV
    if (!data.rc_sdk_control_switch) {
      VLOG(1) << "Switching to Manual UAV state";
      logic_state_machine.process_event(ManualControlEvent());
      return false;
    } else if (data.batterypercent <
               robot_system.getConfiguration().minimum_battery_percent()) {
      LOG(WARNING) << "Battery too low! " << data.batterypercent << "%";
      logic_state_machine.process_event(be::Land());
      return false;
    }
    return true;
  }
};

/**
* @brief Hovering state that uses internal action
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using Hovering_ = BaseState<UAVSystem, LogicStateMachineT,
                            HoveringInternalActionFunctor_<LogicStateMachineT>>;
