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
  using VisualServoing = VisualServoing_<LogicStateMachineT>;

  // Basic transition Actions
  /**
  * @brief Action to take when starting visual servoing
  */
  using VisualServoingTransitionAction =
      VisualServoingTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Check whether visual servoing is feasible currently
  */
  using VisualServoingTransitionGuard =
      VisualServoingTransitionGuardFunctor_<LogicStateMachineT>;

  using GoHomeTransitionAction =
      GoHomeTransitionActionFunctor_<LogicStateMachineT>;
  using GoHomeTransitionGuard =
      GoHomeTransitionGuardFunctor_<LogicStateMachineT>;
};
