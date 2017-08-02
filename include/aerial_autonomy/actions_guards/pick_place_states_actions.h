#pragma once
/// States and actions corresponding to pick and place application
/// using visual servoing.
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/pick_place_functors.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/arm_events.h>
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
  * @brief State during picking an object
  */
  using PickState = PickState_<LogicStateMachineT>;
  // Transition Actions
  /**
  * @brief Action to poweroff arm
  */
  using ArmPowerOn = ArmPoweronTransitionActionFunctor_<LogicStateMachineT>;
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
      typename vsa::VisualServoingTransitionAction,
      VisualServoingArmTransitionActionFunctor_<LogicStateMachineT>>>;
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
  * @brief Guard to stop pick place if arm is not powered or vision is not
  * working
  */
  using PickTransitionGuard = bAnd<typename vsa::VisualServoingTransitionGuard,
                                   ArmEnabledGuardFunctor_<LogicStateMachineT>>;
  /**
  * @brief Action to grab an object
  */
  using PickGuard = PickGuard_<LogicStateMachineT>;
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
              msmf::Internal<arm_events::RightAngleFold, ArmRightFold,
                             msmf::none>> {};
  };
};
