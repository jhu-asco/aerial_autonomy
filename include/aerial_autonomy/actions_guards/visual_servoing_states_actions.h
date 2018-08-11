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
  * @brief State when reaching a visual servoing goal
  */
  using VisualServoing =
      VisualServoing_<LogicStateMachineT, be::Abort,
                      VisualServoingControllerDroneConnector>;

  /**
  * @brief State when reaching a relative pose visual servoing goal using rpyt
  * controller
  */
  using RPYTRelativePoseVisualServoing =
      VisualServoing_<LogicStateMachineT, be::Abort,
                      RPYTRelativePoseVisualServoingConnector>;

  /**
  * @brief State when reaching a relative pose visual servoing goal using MPC
  * controller
  */
  using MPCRelativePoseVisualServoing =
      VisualServoing_<LogicStateMachineT, be::Abort,
                      MPCControllerQuadConnector>;
  // Basic transition Actions
  /**
  * @brief Action to take when starting rpyt relative pose visual servoing
  */
  using RPYTRelativePoseVisualServoingTransitionAction =
      RelativePoseVisualServoingTransitionActionFunctor_<
          LogicStateMachineT, RPYTRelativePoseVisualServoingConnector, 0>;

  /**
  * @brief Action to take when starting mpc relative pose visual servoing
  */
  using MPCRelativePoseVisualServoingTransitionAction =
      RelativePoseVisualServoingTransitionActionFunctor_<
          LogicStateMachineT,
          UAVVisionSystem::VisualServoingReferenceConnectorT, 0>;

  /**
  * @brief Action for initializing relative pose visual servoing
  */
  using ResetRelativePoseVisualServoing =
      ResetRelativePoseVisualServoingTransitionActionFunctor_<
          LogicStateMachineT>;

  /**
  * @brief Check whether visual servoing is feasible currently
  */
  using VisualServoingTransitionGuard =
      bAnd<InitializeTrackerGuardFunctor_<LogicStateMachineT,
                                          ClosestTrackingStrategy>,
           VisualServoingTransitionGuardFunctor_<LogicStateMachineT>>;

  /**
  * @brief Check whether relative pose visual servoing is feasible currently
  */
  using RelativePoseVisualServoingTransitionGuard =
      bAnd<InitializeTrackerGuardFunctor_<LogicStateMachineT,
                                          ClosestTrackingStrategy>,
           CheckGoalIndex_<LogicStateMachineT, 0>>;
};
