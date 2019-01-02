#pragma once
// States and actions corresponding to basic events
#include <aerial_autonomy/actions_guards/uav_states_actions.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/trackers/closest_tracking_strategy.h>
#include <boost/msm/front/euml/operator.hpp>

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct VisualServoingStatesActions : UAVStatesActions<LogicStateMachineT> {
  /**
   * @brief Logical and functor between two guard functions
   *
   * @tparam G1 First guard functor
   * @tparam G2 Second guard functor
   */
  template <class G1, class G2>
  using bAnd = boost::msm::front::euml::And_<G1, G2>;

  // Visual servoing states
  /**
  * @brief State when reaching a relative pose visual servoing goal
  */
  using RelativePoseVisualServoing =
      VisualServoing_<LogicStateMachineT, be::Abort>;

  // Basic transition Actions
  /**
  * @brief Action to take when starting rpyt relative pose visual servoing
  */
  using RelativePoseVisualServoingTransitionAction =
      RelativePoseVisualServoingTransitionActionFunctor_<LogicStateMachineT, 0>;

  /**
  * @brief Action for initializing relative pose visual servoing
  */
  using ResetRelativePoseVisualServoing =
      ResetRelativePoseVisualServoingTransitionActionFunctor_<
          LogicStateMachineT>;

  /**
  * @brief Check whether relative pose visual servoing is feasible currently
  */
  using RelativePoseVisualServoingTransitionGuard =
      bAnd<InitializeTrackerGuardFunctor_<LogicStateMachineT,
                                          ClosestTrackingStrategy>,
           CheckGoalIndex_<LogicStateMachineT, 0>>;

  /**
   * @brief Check whether heading depth visual servoing is feasible and sets
   * goal also
   */
  using VisualServoingTransitionGuard =
      VisualServoingTransitionGuardFunctor_<LogicStateMachineT>;
};
