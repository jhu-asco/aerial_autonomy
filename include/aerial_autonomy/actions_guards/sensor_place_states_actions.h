#pragma once
/// States and actions corresponding to sensor placement applications
/// using visual servoing.
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/arm_states_actions.h>
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/actions_guards/sensor_place_functors.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/arm_events.h>
#include <aerial_autonomy/pick_place_events.h>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>

// State index list
#define preplace_index 0
#define place_index 1
#define checking_index 2
#define postplace_index 0

#define place_arm_index 0
#define checking_arm_index 1

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct SensorPlaceStatesActions
    : VisualServoingStatesActions<LogicStateMachineT>,
      ArmStatesActions<LogicStateMachineT> {
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
  /**
   * @brief namespace for states and actions for arm functors
   */
  using asa = ArmStatesActions<LogicStateMachineT>;
  /**
   * @brief namespace for states and actions for pick place functors
   */
  using psa = PickPlaceStatesActions<LogicStateMachineT>;

  // Sensor Placement States
  /**
  * @brief State while positioning the uav for pre-placing
  */
  using PreSensorPlaceState = PreSensorPlaceState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for placing
  */
  using SensorPlaceState = SensorPlaceState_<LogicStateMachineT>;
  /**
  * @brief State while pulling away from a placement.
  */
  using SensorCheckingState = SensorCheckingState_<LogicStateMachineT>;
  /**
  * @brief State for retreating after placing object
  */
  using PostPlaceState = PostPlaceState_<LogicStateMachineT>;

  // Transition Actions and Guards
  /**
  * @brief Action to take when starting placing.
  */
  using PreSensorPlaceVisualServoingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          typename vsa::ResetRelativePoseVisualServoing,
          typename asa::ArmRightFold,
          ResetThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, preplace_index, true>>>; // Set Home set to
                                                           // true

  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
  using PreSensorPlaceVisualServoingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, preplace_index>;

  /**
  * @brief Action to take when placing sensor
  */
  using SensorPlaceVisualServoingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          ArmPoseTransitionActionFunctor_<LogicStateMachineT, place_arm_index,
                                          false, false>,
          typename vsa::ResetRelativePoseVisualServoing,
          ZeroThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, place_index, false>>>; // Don't set home

  /**
  * @brief Guard to take when placing sensor. Might be nothing.
  */
  using SensorPlaceVisualServoingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, place_index>;

  /**
  * @brief Action to take when checking sensor placement
  */
  using SensorCheckingVisualServoingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          ArmPoseTransitionActionFunctor_<LogicStateMachineT,
                                          checking_arm_index, false, false>,
          typename vsa::ResetRelativePoseVisualServoing,
          ZeroThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, checking_index, false>>>; // Don't set home

  /**
  * @brief Guard to take when checking sensor placement. Might be nothing.
  */
  using SensorCheckingVisualServoingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, checking_index>;

  // Transition Actions and Guards
  /**
  * @brief Action to take after checking and when returning to the staging
  * location.
  */
  using PostPlaceVisualServoingTransitionAction =
      base_functors::bActionSequence<boost::mpl::vector<
          ArmGripActionFunctor_<LogicStateMachineT, false>,
          typename vsa::ResetRelativePoseVisualServoing,
          typename asa::ArmRightFold,
          ResetThrustMixingGain_<LogicStateMachineT>,
          RelativePoseVisualServoingTransitionActionFunctor_<
              LogicStateMachineT, postplace_index, false>>>; // Don't set home

  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
  using PostPlaceVisualServoingTransitionGuard =
      CheckGoalIndex_<LogicStateMachineT, postplace_index>;
};
