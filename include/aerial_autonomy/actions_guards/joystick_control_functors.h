#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/empty_goal.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <aerial_autonomy/types/manual_control_event.h>
#include <glog/logging.h>
#include <parsernode/common.h>

template<class LogicStateMachineT, class AbortEventT=ManualControlEvent>
struct JoystickControlInternalActionFunctor_
: HoveringInternalActionFunctor_<LogicStateMachineT, AbortEventT>{
};
/**
*
* @ brief Action while transitioning to joystick control
*
* @ tparam LogicStateMachineT logic state machine to process events 
*/
template<class LogicStateMachineT>
struct JoystickControlTransitionActionFunctor_
: EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT>{
  void run(UAVSystem &robot_system){
    LOG(WARNING) << "entering manual velocity mode";
    robot_system.setGoal<ManualVelocityControllerDroneConnector, EmptyGoal>(
      EmptyGoal());
  }
};
/**
* 
*@ brief Joystick control state that uses internal action
*
* @ tparam LogicStateMachineT logic state machine to process events 
*/
template<class LogicStateMachineT>
using JoystickControl_ = BaseState<UAVSystem, LogicStateMachineT,
JoystickControlInternalActionFunctor_<LogicStateMachineT>>;
