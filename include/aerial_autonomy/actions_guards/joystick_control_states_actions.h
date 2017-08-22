#pragma once
#include <aerial_autonomy/actions_guards/uav_states_actions.h>
#include <aerial_autonomy/actions_guards/joystick_control_functors.h>

/**
* @ brief 
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct JoystickControlStatesActions : UAVStatesActions<LogicStateMachineT>{
  /**
  * @ brief  State when in Joystick control mode 
  */
  using JoystickControlState = JoystickControl_<LogicStateMachineT>;
  /**
  * @ brief Action while transitioning to Joystick control state
  */
  using JoystickControlAction = JoystickControlTransitionActionFunctor_<LogicStateMachineT>;
};
