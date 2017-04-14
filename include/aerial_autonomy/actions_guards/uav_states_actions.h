// States and actions corresponding to basic events
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/land_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/position_control_functors.h>
#include <aerial_autonomy/actions_guards/takeoff_functors.h>

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT> struct UAVStatesActions {
  // Basic states
  /**
  * @brief Takingoff state
  */
  using TakingOff = TakingOff_<LogicStateMachineT>;
  /**
  * @brief Landing state
  */
  using Landing = Landing_<LogicStateMachineT>;
  /**
  * @brief Reaching goal state
  */
  using ReachingGoal = ReachingGoal_<LogicStateMachineT>;
  /**
  * @brief Hovering state
  */
  using Hovering = Hovering_<LogicStateMachineT>;
  /**
  * @brief Landed state
  */
  using Landed = Landed_<LogicStateMachineT>;
  /**
  * @brief Manual control state
  */
  using ManualControlState = ManualControlState_<LogicStateMachineT>;

  // Basic transition Actions
  /**
  * @brief Action to take when taking off
  */
  using TakeoffAction = TakeoffTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Guard to stop taking off under low voltage
  */
  using TakeoffGuard = TakeoffTransitionGuardFunctor_<LogicStateMachineT>;
  /**
  * @brief Abort action when taking off
  */
  using TakeoffAbort = TakeoffAbortActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Action to take when landing
  */
  using LandingAction = LandTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief set goal action when transitioning
  */
  using ReachingGoalSet =
      PositionControlTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Guard to avoid going to goal if goal is not correct
  */
  using ReachingGoalGuard =
      PositionControlTransitionGuardFunctor_<LogicStateMachineT>;
  /**
  * @brief Abort action when reaching goal
  */
  using UAVControllerAbort =
      UAVControllerAbortActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Land action when reaching goal
  */
  using ReachingGoalLand = LandingAction;
  /**
  * @brief Action to enable sdk when switching to sdk mode
  */
  using ManualControlSwitchAction =
      ManualControlSwitchAction_<LogicStateMachineT>;
  /**
  * @brief Guard for switching to SDK mode from manual control
  */
  using ManualControlSwitchGuard =
      ManualControlSwitchGuard_<LogicStateMachineT>;
};
