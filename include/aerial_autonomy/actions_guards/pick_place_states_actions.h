#pragma once
/// States and actions corresponding to pick and place application
/// using visual servoing.
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/pick_place_functors.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <boost/msm/front/euml/operator.hpp>

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct PickPlaceStatesActions
    : VisualServoingStatesActions<LogicStateMachineT> {
  // Pre takeoff, land states
  /**
  * @brief State before landing
  */
  using ArmFolding = ArmFolding_<LogicStateMachineT>;
  // PickPlace State
  /**
  * @brief State during picking an object
  */
  using PickState = PickState_<LogicStateMachineT>;
  // Manual control along with arm status check state
  /**
  * @brief State to check for arm power, manual rc switch
  */
  using ManualControlArmState = ManualControlArmState_<LogicStateMachineT>;

  // Transition Actions
  /**
  * @brief Action to poweroff arm
  */
  using ArmPoweron = ArmPoweronTransitionActionFunctor_<LogicStateMachineT>;
  /**
  * @brief Action to poweroff arm
  */
  using ArmPoweroff = ArmPoweroffTransitionActionFunctor_<LogicStateMachineT>;
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
  using AbortUAVControllerArmRightFold = msm::front::euml::ActionSequence<
      boost::mpl::vector<UAVControllerAbort, ArmRightFoldTransitionAction>>;
  /**
  * @brief Action to take when starting folding arm before takeoff
  */
  using ArmPoweronFold =
      msm::front::euml::ActionSequence<boost::mpl::vector<ArmPoweron, ArmFold>>;
  /**
  * @brief Action sequence to abort UAV controller and arm controller
  * angle
  */
  using AbortUAVArmController = msm::front::euml::ActionSequence<
      boost::mpl::vector<UAVControllerAbort, AbortArmController>>;
  // Guards
  /**
  * @brief Guard to stop pick place if arm is not powered or vision is not
  * working
  */
  using PickTransitionGuard = PickTransitionGuardFunctor_<LogicStateMachineT>;
  /**
  * @brief Action to grab an object
  */
  using PickAction = PickAction_<LogicStateMachineT>;
};
