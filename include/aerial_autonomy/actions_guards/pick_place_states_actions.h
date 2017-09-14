#pragma once
/// States and actions corresponding to pick and place application
/// using visual servoing.
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/pick_place_functors.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/arm_events.h>
#include <aerial_autonomy/pick_place_events.h>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct PickPlaceStatesActions
    : VisualServoingStatesActions<LogicStateMachineT> {

  /**
   * @brief  Action sequence to chain multiple actions together
   *
   * @tparam Sequence sequence of actions
   */
  template <class Sequence>
  using bActionSequence = boost::msm::front::ActionSequence_<Sequence>;
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

  // Pre takeoff, land states
  /**
  * @brief State for folding arm
  */
  using ArmFolding = ArmFolding_<LogicStateMachineT>;
  /**
  * @brief State before takeoff
  */
  using ArmPreTakeoffFolding = ArmPreTakeoffFolding_<LogicStateMachineT>;
  /**
  * @brief State before landing
  */
  using ArmPreLandingFolding = ArmPreLandingFolding_<LogicStateMachineT>;
  // PickPlace State
  /**
  * @brief State while picking an object
  */
  using PickState = PickState_<LogicStateMachineT>;
  // PrePickPlace State
  /**
  * @brief State while positioning the arm for picking
  */
  using PrePickState = PrePickState_<LogicStateMachineT>;
  /**
  * @brief State while positioning the uav for placing
  */
  using PlaceState = PlaceState_<LogicStateMachineT>;
  // Transition Actions
  /**
  * @brief Action to poweroff arm
  */
  using ArmPowerOn = ArmPoweronTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Action to grip/ungrip gripper on arm
  */
  template <bool grip>
  using ArmGripAction = ArmGripActionFunctor_<LogicStateMachineT, grip>;
  /**
  * @brief Action to poweroff arm
  */
  using ArmPowerOff = ArmPoweroffTransitionActionFunctor_<LogicStateMachineT>;

  /**
  * @brief Check if post pick waypoints are specified in proto config
  */
  using PostPickWaypointGuard =
      WaypointSequenceTransitionGuardFunctor_<LogicStateMachineT, 0, 1>;
  /**
  * @brief Check if post place waypoints arespecified in proto config
  */
  using PostPlaceWaypointGuard =
      WaypointSequenceTransitionGuardFunctor_<LogicStateMachineT, 2, 3>;
  /**
  * @brief Action to take when starting folding arm before land
  */
  using ArmFold = ArmFoldTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Action to fold arm into right angle configuration
  */
  using ArmRightFold = ArmRightFoldTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Abort arm controller
  */
  using AbortArmController = AbortArmController_<LogicStateMachineT>;

  /**
  * @brief Action sequence that ungrips then goes home
  */
  using UngripGoHome =
      bActionSequence<boost::mpl::vector<ArmGripAction<false>,
                                         typename vsa::GoHomeTransitionAction>>;
  /**
  * @brief Action sequence to abort UAV controller and move arm to right
  * angle
  */
  using AbortUAVControllerArmRightFold = bActionSequence<
      boost::mpl::vector<typename usa::UAVControllerAbort, ArmRightFold>>;
  /**
  * @brief Action sequence to abort UAV controller and move arm to right
  * angle
  */
  using AbortUAVControllerArmFold = bActionSequence<
      boost::mpl::vector<typename usa::UAVControllerAbort, ArmFold>>;
  /**
  * @brief Set goal for visual servoing and also arm controller
  */
  using PickTransitionAction = bActionSequence<boost::mpl::vector<
      AbortArmController, RelativePoseVisualServoingTransitionActionFunctor_<
                              LogicStateMachineT, 1, false>>>;
  /**
  * @brief Move arm to pre-pick pose
  */
  using PrePickTransitionAction =
      ArmPoseTransitionActionFunctor_<LogicStateMachineT, 0>;
  /**
  * @brief Check tracking is valid and arm is enabled for pre-pick
  */
  using PrePickTransitionGuard =
      PrePickTransitionGuardFunctor_<LogicStateMachineT>;

  /**
  * @brief Action to take when starting placing object.
  */
  using PlaceVisualServoingTransitionAction =
      RelativePoseVisualServoingTransitionActionFunctor_<LogicStateMachineT, 2>;

  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
  using PlaceVisualServoingTransitionGuard =
      ExplicitIdVisualServoingGuardFunctor_<LogicStateMachineT, 15>;

  /**
  * @brief Action to take when starting folding arm before takeoff
  */
  using ArmPowerOnFold =
      bActionSequence<boost::mpl::vector<ArmPowerOn, ArmFold>>;
  /**
  * @brief Action sequence to abort UAV controller and arm controller
  * angle
  */
  using AbortUAVArmController = bActionSequence<
      boost::mpl::vector<typename usa::UAVControllerAbort, AbortArmController>>;
  // Guards
  /**
  * @brief Guard to stop pick place if arm is not powered
  */
  using PickGuard = bAnd<PickTransitionGuardFunctor_<LogicStateMachineT>,
                         ArmEnabledGuardFunctor_<LogicStateMachineT>>;

  /**
  * @brief State for following waypoints after picking object
  */
  using ReachingPostPickWaypoint =
      FollowingWaypointSequence_<LogicStateMachineT, 0, 1>;
  /**
  * @brief State for following waypoints after placing object
  */
  using ReachingPostPlaceWaypoint =
      FollowingWaypointSequence_<LogicStateMachineT, 2, 3>;

  // Explicitly defined manual Control state
  /**
  * @brief State that checks arm status along with regular manual control
  * state
  *
  * @tparam LogicStateMachineT Logic state machine used to process events
  */
  struct ManualControlArmState : public msmf::state<> {
    struct internal_transition_table
        : boost::mpl::vector<
              msmf::Internal<
                  InternalTransitionEvent,
                  ManualControlArmInternalActionFunctor_<LogicStateMachineT>,
                  msmf::none>,
              msmf::Internal<arm_events::PowerOn, ArmPowerOn, msmf::none>,
              msmf::Internal<arm_events::PowerOff, ArmPowerOff, msmf::none>,
              msmf::Internal<arm_events::Fold, ArmFold, msmf::none>,
              msmf::Internal<arm_events::Grip, ArmGripAction<true>, msmf::none>,
              msmf::Internal<arm_events::UnGrip, ArmGripAction<false>,
                             msmf::none>,
              msmf::Internal<uav_basic_events::Abort, msmf::none, msmf::none>,
              msmf::Internal<arm_events::RightAngleFold, ArmRightFold,
                             msmf::none>> {};
  };
};
