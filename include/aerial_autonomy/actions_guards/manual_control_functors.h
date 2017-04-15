#pragma once

#include <aerial_autonomy/robot_systems/uav_system.h>

#include <aerial_autonomy/types/manual_control_event.h>
#include <aerial_autonomy/uav_basic_events.h>

// Boost Includes
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <parsernode/common.h>

/**
* @brief Basic events such as takeoff land etc
*/
namespace be = uav_basic_events;

/**
* @brief action to take when switching from manual UAV to SDK Mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ManualControlSwitchAction_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    VLOG(1) << "Enabling SDK";
    robot_system.enableAutonomousMode();
  }
};

/**
* @brief Guard to avoid switching to SDK mode unless hardware allows
* it.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ManualControlSwitchGuard_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system, LogicStateMachineT &) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    if (data.batterypercent <
        robot_system.getConfiguration().minimum_battery_percent()) {
      LOG(WARNING) << "Battery too low! " << data.batterypercent;
      return false;
    }
    return data.rc_sdk_control_switch;
  }
};

/**
* @brief Check if hardware is allowing to switch sdk mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ManualControlInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Check if hardware is allowing to switch to sdk mode
  * and trigger appropriate events
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // If sdk mode is allowed
    if (data.rc_sdk_control_switch) {
      if (data.localpos.z < robot_system.getConfiguration().landing_height()) {
        VLOG(1) << "Switching to Landed mode";
        logic_state_machine.process_event(be::Land());
      } else {
        VLOG(1) << "Switching to Hovering mode";
        logic_state_machine.process_event(be::Takeoff());
      }
    }
  }
};

/**
* @brief State that uses internal action for switching out of sdk mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ManualControlState_ =
    BaseState<UAVSystem, LogicStateMachineT,
              ManualControlInternalActionFunctor_<LogicStateMachineT>>;
