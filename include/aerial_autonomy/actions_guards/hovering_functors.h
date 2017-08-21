#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/manual_control_event.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <glog/logging.h>
#include <parsernode/common.h>

namespace be = uav_basic_events;

/**
* @brief Internal action when hovering.
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam AbortEventT Event to generate when aborting (Default
* ManualControlEvent)
*/
template <class LogicStateMachineT, class AbortEventT = ManualControlEvent>
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
      logic_state_machine.process_event(AbortEventT());
      return false;
    } else if (data.batterypercent <
               robot_system.getConfiguration().minimum_battery_percent()) {
      LOG(WARNING) << "Battery too low! " << data.batterypercent
                   << "\% Landing!";
      logic_state_machine.process_event(be::Land());
      return false;
    }
    return true;
  }
};

/**
* @brief Logic to abort if controller status is critical
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam ControllerConnector Controller connector whose status the functor
* checks
* @tparam check_completed If false, ignores check on controller completion
*/
template <class LogicStateMachineT, class ControllerConnector,
          bool check_completed = true>
struct ControllerStatusInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief check if controller status is critical and abort;
  * similarly if controller status is completed raise completed
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    // Check status of controller
    ControllerStatus status = robot_system.getStatus<ControllerConnector>();
    if (status == ControllerStatus::Completed && check_completed) {
      VLOG(1) << "Reached goal for " << typeid(ControllerConnector).name();
      logic_state_machine.process_event(Completed());
      return false;
    } else if (status == ControllerStatus::Critical) {
      LOG(WARNING) << "Controller critical for "
                   << typeid(ControllerConnector).name();
      logic_state_machine.process_event(be::Abort());
      return false;
    }
    return true;
  }
};

/**
 * @brief Create a action functor that checks the UAV status and aborts if
 * UAV is in manual control mode.
 *
 * @tparam LogicStateMachineT
 */
template <class LogicStateMachineT>
using UAVStatusInternalActionFunctor_ =
    HoveringInternalActionFunctor_<LogicStateMachineT, be::Abort>;
/**
* @brief Hovering state that uses internal action
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using Hovering_ = BaseState<UAVSystem, LogicStateMachineT,
                            HoveringInternalActionFunctor_<LogicStateMachineT>>;
