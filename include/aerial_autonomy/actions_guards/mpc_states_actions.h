#pragma once
#include <aerial_autonomy/actions_guards/arm_states_actions.h>
#include <aerial_autonomy/actions_guards/mpc_functors.h>
#include <aerial_autonomy/actions_guards/uav_states_actions.h>

template <class LogicStateMachineT>
struct MPCStatesActions : UAVStatesActions<LogicStateMachineT>,
                          ArmStatesActions<LogicStateMachineT> {
  /**
   * @brief  Action sequence to chain multiple actions together
   *
   * @tparam Sequence sequence of actions
   */
  template <class Sequence>
  using bActionSequence = boost::msm::front::ActionSequence_<Sequence>;
  /**
   * @brief namespace for states and actions for basic uav actions
   */
  using usa = UAVStatesActions<LogicStateMachineT>;
  /**
   * @brief namespace for states and actions for arm functors
   */
  using asa = ArmStatesActions<LogicStateMachineT>;
  /// States
  /**
   * @brief MPC state
   */
  using MPCState = MPCState_<LogicStateMachineT>;

  /// Transition Actions
  /**
   * @brief MPC transition action for tracking spiral reference
   */
  using MPCSpiralTransition =
      MPCSpiralReferenceTrackingTransition<LogicStateMachineT>;
  /**
   * @brief MPC transition action for tracking waypoint
   */
  using MPCWaypointTransition =
      MPCWaypointReferenceTrackingTransition<LogicStateMachineT>;
  /**
  * @brief Action sequence to abort arm controllers and move arm to
  * right angle
  */
  using AbortArmControllerArmRightFold =
      bActionSequence<boost::mpl::vector<typename asa::AbortArmController,
                                         typename asa::ArmRightFold>>;
  /**
  * @brief Action sequence to abort UAV and arm controllers and move arm to
  * right angle
  */
  using AbortUAVArmControllerArmRightFold =
      bActionSequence<boost::mpl::vector<typename usa::UAVControllerAbort,
                                         AbortArmControllerArmRightFold>>;
};
