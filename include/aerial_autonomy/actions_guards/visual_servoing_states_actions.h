#pragma once
// States and actions corresponding to basic events
#include <aerial_autonomy/actions_guards/uav_states_actions.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct VisualServoingStatesActions : UAVStatesActions<LogicStateMachineT> {
  // Visual servoing states
  /**
  * @brief State when reaching a visual servoing goal
  */
  using VisualServoing = VisualServoing_<LogicStateMachineT>;

  /**
  * @brief State when reaching a relative pose visual servoing goal
  */
  using RelativePoseVisualServoing =
      RelativePoseVisualServoing_<LogicStateMachineT>;

  // Basic transition Actions
  /**
  * @brief Action to take when starting visual servoing
  */
  using VisualServoingTransitionAction =
      VisualServoingTransitionActionFunctor_<LogicStateMachineT>;

  /**
  * @brief Action to take when starting relative pose visual servoing
  */
  using RelativePoseVisualServoingTransitionAction =
      RelativePoseVisualServoingTransitionActionFunctor_<LogicStateMachineT, 0>;

  /**
  * @brief Check whether visual servoing is feasible currently
  */
  using VisualServoingTransitionGuard =
      VisualServoingTransitionGuardFunctor_<LogicStateMachineT>;

  /**
  * @brief Check whether relative pose visual servoing is feasible currently
  */
  using RelativePoseVisualServoingTransitionGuard =
      RelativePoseVisualServoingTransitionGuardFunctor_<LogicStateMachineT>;

  /**
  * @brief Send the UAV back to home position
  */
  using GoHomeTransitionAction =
      GoHomeTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Check if home location is specified
  */
  using GoHomeTransitionGuard =
      GoHomeTransitionGuardFunctor_<LogicStateMachineT>;
};
