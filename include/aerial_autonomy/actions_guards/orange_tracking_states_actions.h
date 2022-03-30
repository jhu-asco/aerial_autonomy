#pragma once
/// States and actions corresponding to sensor placement applications
/// using visual servoing.
//#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/actions_guards/orange_tracking_functors.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/actions_guards/position_control_functors.h>
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/orange_tracking_events.h>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>

#include <aerial_autonomy/trackers/closest_tracking_strategy.h>

// State index list
#define prepick_index 0
#define pick_index 1
#define rise_index 2

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct OrangeTrackingStatesActions
    : VisualServoingStatesActions<LogicStateMachineT>{
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
  using vsa = VisualServoingStatesActions<LogicStateMachineT>;
  /**
   * @brief namespace for states and actions for basic uav actions
   */
  using usa = UAVStatesActions<LogicStateMachineT>;

  // Orange Tracking States
  /**
  * @brief State while positioning the uav for pre-picking
  */
  using PreOrangeTrackingState = PreOrangeTrackingState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for picking
  */
  using OrangeTrackingState = OrangeTrackingState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for picking
  */
  using ResetOrangeTracking = ResetOrangeTrackingState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for picking
  */
  using ResetTrialState = ResetTrialState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for picking
  */
  using OrangeTrackingFinalRiseState = OrangeTrackingFinalRiseState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for picking
  */
  using OrangeGrippingState = OrangeGrippingState_<LogicStateMachineT>;

  // Transition Actions and Guards
  /**
  * @brief Action to take when starting placing.
  */
  using ResetTrialTransitionAction = 
      base_functors::bActionSequence<boost::mpl::vector<
          ResetTrialTransitionActionFunctor_<LogicStateMachineT>,
	  TrialGripTransitionActionFunctor_<LogicStateMachineT>>>;

  /**
  * @brief Action to take when starting placing.
  */
  using PreOrangeTrackingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          SetNoisePolynomialReference_<LogicStateMachineT,false>,
          typename vsa::ResetRelativePoseVisualServoing,
          //ResetThrustMixingGain_<LogicStateMachineT>,
          ArmGripActionFunctor_<LogicStateMachineT,false>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, prepick_index, true>>>; // Set Home set to
                                                           // true

  // Transition Actions and Guards
  /**
  * @brief Action to take when starting placing.
  */
  using PostResetPreOrangeTrackingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          SetNoisePolynomialReference_<LogicStateMachineT,false>,
          typename vsa::ResetRelativePoseVisualServoing,
          //ResetThrustMixingGain_<LogicStateMachineT>,
          ArmGripActionFunctor_<LogicStateMachineT,false>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, prepick_index, false>>>; // Set Home set to
                                                           // true

  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
  using PreOrangeTrackingTransitionGuard =
      /*bAnd<*/bAnd<InitializeTrackerGuardFunctor_<LogicStateMachineT,
                                          ClosestTrackingStrategy>,
           //ArmEnabledGuardFunctor_<LogicStateMachineT>>,
           CheckGoalIndex_<LogicStateMachineT, prepick_index>>;

  /**
  * @brief Action to take when placing sensor
  */
  using OrangeTrackingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          SetNoisePolynomialReference_<LogicStateMachineT,false>,
          typename vsa::ResetRelativePoseVisualServoing,
          SetResetLocationTransitionActionFunctor_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, pick_index, false>>>; // Set home

  /**
  * @brief Guard to take when placing sensor. Might be nothing.
  */
  using OrangeTrackingTransitionGuard =
//           bAnd<ArmEnabledGuardFunctor_<LogicStateMachineT>,
                CheckGoalIndex_<LogicStateMachineT, pick_index>;//>;
  /**
  * @brief Action to take when placing sensor
  */
  using OrangeTrackingRiseTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          SetNoisePolynomialReference_<LogicStateMachineT,true>,
          RelativePositionWaypointTransitionActionFunctor_<LogicStateMachineT,rise_index>>>;

  /**
  * @brief Guard to take when placing sensor. Might be nothing.
  */
  using OrangeTrackingRiseTransitionGuard =
      //bAnd<ArmEnabledGuardFunctor_<LogicStateMachineT>,
           CheckGoalIndex_<LogicStateMachineT, rise_index>;//>;

  /**
  * @brief Action to take when resetting pick.
  */
  using ResetTrackingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          GoResetTransitionActionFunctor_<LogicStateMachineT>,
          ArmGripActionFunctor_<LogicStateMachineT,false>>>;

  /**
  * @brief Action to take when gripping.
  */
  using OrangeGrippingTransitionAction =
      //base_functors::bActionSequence<boost::mpl::vector<
          ArmGripActionFunctor_<LogicStateMachineT,true>;//>>;
};
