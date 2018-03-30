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
  /**
  * @brief State while positioning the uav for placing
  */
  using PlaceState = PlaceState_<LogicStateMachineT>;
  /**
  * @brief State when reaching a relative pose visual servoing goal
  */
  using RelativePoseVisualServoing =
      RelativePoseVisualServoing_<LogicStateMachineT, Reset>;
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
   * @brief Action sequence to ungrip object and go to home location and
   * fold the arm to rightangle
   */
  using RightArmUngripGoHome =
      bActionSequence<boost::mpl::vector<UngripGoHome, ArmRightFold>>;

  /**
  * @brief Action sequence that ungrips then goes home
  */
  using ArmRightFoldGoHome =
      bActionSequence<boost::mpl::vector<AbortArmController, ArmRightFold,
                                         typename vsa::GoHomeTransitionAction>>;
  /**
  * @brief Action sequence that folds to right angle and aborts arm controller
  */
  using AbortArmControllerRightFold =
      bActionSequence<boost::mpl::vector<AbortArmController, ArmRightFold>>;

  /**
  * @brief Action sequence to abort UAV controller and move arm to right
  * angle
  */
  using AbortUAVControllerArmRightFold = bActionSequence<
      boost::mpl::vector<typename usa::UAVControllerAbort, ArmRightFold>>;
  /**
  * @brief Action sequence to abort UAV and arm controllers and move arm to
  * right
  * angle
  */
  using AbortUAVArmControllerArmRightFold =
      bActionSequence<boost::mpl::vector<typename usa::UAVControllerAbort,
                                         AbortArmController, ArmRightFold>>;
  /**
  * @brief Action sequence to abort UAV controller and move arm to right
  * angle
  */
  using AbortUAVControllerArmFold = bActionSequence<
      boost::mpl::vector<typename usa::UAVControllerAbort, ArmFold>>;

  /**
  * @brief Check tracking is valid and arm is enabled for pick
  */
  using PickTransitionGuard =
      bAnd<typename vsa::RelativePoseVisualServoingTransitionGuard,
           ArmTrackingGuardFunctor_<LogicStateMachineT>>;
  // \todo Check if tracked object is in workspace

  /**
  * @brief Move arm to pick pose and move to tracked object
  */
  using PickTransitionAction = bActionSequence<
      boost::mpl::vector<ArmPoseTransitionActionFunctor_<LogicStateMachineT, 0>,
                         typename vsa::ResetRelativePoseVisualServoing,
                         RelativePoseVisualServoingTransitionActionFunctor_<
                             LogicStateMachineT, 0>>>;

  /**
  * @brief Action to take when starting placing object at either drop-off.
  */
  using PlaceVisualServoingTransitionAction = bActionSequence<
      boost::mpl::vector<typename vsa::ResetRelativePoseVisualServoing,
                         RelativePoseVisualServoingTransitionActionFunctor_<
                             LogicStateMachineT, 1>>>;
  // \todo Matt add guard to check if relative pose visual servoing goal exists

  /**
  * @brief Guard to set and check that the id to track is available
  * before beginning visual servoing
  */
  using PlaceVisualServoingTransitionGuard =
      EventIdVisualServoingGuardFunctor_<LogicStateMachineT>;

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
  /**
  * @brief State for following waypoints after picking object
  */
  using ReachingPostPickWaypoint =
      ReachingPostPickWaypoint_<LogicStateMachineT, 0, 0>;
  /**
  * @brief Base state for following waypoints after picking object.
  *  Used for setting state config
  */
  using ReachingPostPickWaypointBase =
      FollowingWaypointSequence_<LogicStateMachineT, 0, 0, ObjectId>;

  /**
   * @brief State to wait for picking
   */
  using WaitingForPick = WaitingForPick_<LogicStateMachineT>;

  /**
  * @brief State for following waypoints after placing object
  */
  using ReachingPostPlaceWaypoint =
      FollowingWaypointSequence_<LogicStateMachineT, 2, 3>;
  /**
  * @brief State for resetting visual servoing
  */
  struct ResetVisualServoing : ReachingGoal_<LogicStateMachineT> {};

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
