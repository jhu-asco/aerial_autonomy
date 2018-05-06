#pragma once
// States and actions corresponding to basic events
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/land_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/position_control_functors.h>
#include <aerial_autonomy/actions_guards/takeoff_functors.h>
#include <aerial_autonomy/actions_guards/velocity_control_functors.h>

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
  using ReachingGoal1 = ReachingGoal1_<LogicStateMachineT>;
  using ReachingGoal2 = ReachingGoal2_<LogicStateMachineT>;
  using ReachingGoal3 = ReachingGoal3_<LogicStateMachineT>;
  using ReachingGoal4 = ReachingGoal4_<LogicStateMachineT>;


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
  /**
  * @brief Track commanded velocity
  */
  using ExecutingVelocityGoal = ExecutingVelocityGoal_<LogicStateMachineT>;

  /**
  * @brief State for running joystick velocity controller
  */
  using RunningJoystickVelocityController =
      RunningJoystickVelocityController_<LogicStateMachineT>;

  /**
  * @brief State for running joystick rpyt controller
  */
  using RunningJoystickRPYTController =
      RunningJoystickRPYTController_<LogicStateMachineT>;

  // Basic transition Actions
  using ReachingGoalSet1 = PositionControlTransitionActionGoal1Functor<LogicStateMachineT>;
  using ReachingGoalSet2 = PositionControlTransitionActionGoal2Functor<LogicStateMachineT>;
  using ReachingGoalSet3 = PositionControlTransitionActionGoal3Functor<LogicStateMachineT>;
  using ReachingGoalSet4 = PositionControlTransitionActionGoal4Functor<LogicStateMachineT>;

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
      PositionControlTransitionActionFunctor_<LogicStateMachineT, RPYTBasedOdomSensorControllerDroneConnector>;
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
  /**
  * @brief set velocity controller goal
  */
  using SetVelocityGoal =
      VelocityControlTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief start joystick velocity controller
  */
  using StartJoystickVelocityController =
      JoystickVelocityControlTransitionActionFunctor_<LogicStateMachineT>;
  /**
   * @brief start joystick rpyt controller
   */
  using StartJoystickRPYTController =
      JoystickRPYTControlActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Guard to avoid going to velocity goal if too high
  */
  using GuardVelocityGoal =
      VelocityControlTransitionGuardFunctor_<LogicStateMachineT>;
};
