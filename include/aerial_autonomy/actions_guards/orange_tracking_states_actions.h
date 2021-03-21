#pragma once
/// States and actions corresponding to sensor placement applications
/// using visual servoing.
//#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/actions_guards/orange_tracking_functors.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/orange_tracking_events.h>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>

#include <aerial_autonomy/trackers/closest_tracking_strategy.h>

// State index list
#define prepick_index 0
#define pick_index 1
//#define checking_index 2
//#define postplace_index 0

//#define place_arm_index 0
//#define checking_arm_index 1

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
  * @brief State while pulling away from a placement.
  */
  //using SensorCheckingState = SensorCheckingState_<LogicStateMachineT>;
  /**
  * @brief State for retreating after placing object
  */
  //using PostPlaceState = PostPlaceState_<LogicStateMachineT>;

  // Transition Actions and Guards
  /**
  * @brief Action to take when starting placing.
  */
  using PreOrangeTrackingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          typename vsa::ResetRelativePoseVisualServoing,
          //ResetThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, prepick_index, true>>>; // Set Home set to
                                                           // true

  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
  using PreOrangeTrackingTransitionGuard =
      bAnd<InitializeTrackerGuardFunctor_<LogicStateMachineT,
                                          ClosestTrackingStrategy>,
           CheckGoalIndex_<LogicStateMachineT, prepick_index>>;

  /**
  * @brief Action to take when placing sensor
  */
  using OrangeTrackingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          typename vsa::ResetRelativePoseVisualServoing,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, pick_index, false>>>; // Don't set home

  /**
  * @brief Guard to take when placing sensor. Might be nothing.
  */
  using OrangeTrackingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, pick_index>;

  /**
  * @brief Action to take when checking sensor placement
  */
/*  using SensorCheckingVisualServoingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          ArmPoseTransitionActionFunctor_<LogicStateMachineT,
                                          checking_arm_index, false, false>,
          typename vsa::ResetRelativePoseVisualServoing,
          ResetAccelerationBiasEstimator_<LogicStateMachineT>,
          ZeroThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, checking_index, false>>>; // Don't set home
*/
  /**
  * @brief Guard to take when checking sensor placement. Might be nothing.
  */
/*  using SensorCheckingVisualServoingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, checking_index>;
*/
  // Transition Actions and Guards
  /**
  * @brief Action to take after checking and when returning to the staging
  * location.
  */
/*  using PostPlaceVisualServoingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          ArmGripActionFunctor_<LogicStateMachineT, false>,
          typename vsa::ResetRelativePoseVisualServoing,
          typename asa::ArmRightFold,
          ResetThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, postplace_index, false>>>; // Don't set home
*/
  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
/*  using PostPlaceVisualServoingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, postplace_index>;*/
};
