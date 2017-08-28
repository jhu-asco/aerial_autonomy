#pragma once
#include <aerial_autonomy/actions_guards/joystick_control_functors.h>
#include <aerial_autonomy/actions_guards/uav_states_actions.h>

/**
* @ brief Class to provide typedefs for joystick control actions and guards
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct JoystickControlStatesActions : UAVStatesActions<LogicStateMachineT> {
  /**
  * @ brief  State when in Joystick control mode
  */
  using JoystickControlState = JoystickControlState_<LogicStateMachineT>;
  /**
  * @ brief Action while transitioning to Joystick control state
  */
  using JoystickControlAction =
      JoystickControlTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Guard to check while transitioning to joystick control state
  */
  using JoystickControlGuard =
      JoystickControlTransitionGuardFunctor_<LogicStateMachineT>;
};
