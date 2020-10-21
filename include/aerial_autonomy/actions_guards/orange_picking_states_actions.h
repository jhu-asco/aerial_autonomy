#pragma once
/// States and actions corresponding to pick and place application
/// using visual servoing.
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/arm_states_actions.h>
#include <aerial_autonomy/actions_guards/pick_place_functors.h>
#include <aerial_autonomy/actions_guards/orange_picking_functors.h>
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/arm_events.h>
#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/orange_picking_events.h>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>

/**
* @brief Class to provide typedefs for orange picking-related states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT1>
struct OrangePickingStatesActions : UAVStatesActions<LogicStateMachineT1> {
  /**
   * @brief Logical and functor between two guard functions
   *
   * @tparam G1 First guard functor
   * @tparam G2 Second guard functor
   */
  template <class G1, class G2>
  using bAnd = boost::msm::front::euml::And_<G1, G2>;

  /**
   * @brief namespace for states and actions in visual servoing
   */
  using vsa = VisualServoingStatesActions<LogicStateMachineT1>;
  /**
   * @brief namespace for states and actions for basic uav actions
   */
  using usa = UAVStatesActions<LogicStateMachineT1>;
  /**
   * @brief namespace for states and actions for arm functors
   */
  using asa = ArmStatesActions<LogicStateMachineT1>;
  /**
   * @brief namespace for states and actions for pick-place functors
   */
  using psa = PickPlaceStatesActions<LogicStateMachineT1>;

  // OrangePicking states
  /**
  * @brief State while following the path
  */
  using PathFollowState = PathFollowState_<LogicStateMachineT1>;

  // Transition Actions and Guards
  /**
  * @brief Action to take when starting the PathState
  */
  using PathFollowTransitionAction = 
      base_functors::bActionSequence<boost::mpl::vector<
          ResetPathFollowingTransitionActionFunctor_<LogicStateMachineT1>,
          typename asa::ArmRightFold,
          ResetThrustMixingGain_<LogicStateMachineT1>,
          PathFollowingTransitionActionFunctor_<LogicStateMachineT1>>>;
  /**
  * @brief Guard to take when starting the PathState
  */
  using PathFollowTransitionGuard = PathFollowingTransitionGuardFunctor_<LogicStateMachineT1>;
};
