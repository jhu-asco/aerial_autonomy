#pragma once
/// States and actions corresponding to arm system
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/uav_arm_functors.h>
#include <aerial_autonomy/arm_events.h>
/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT> struct ArmStatesActions {
  /**
  * @brief State for folding arm
  */
  using ArmFolding = ArmFolding_<LogicStateMachineT>;
  /**
  * @brief Landing state
  */
  using ArmStatus = ArmStatus_<LogicStateMachineT>;
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
   * @brief Start arm sine controller
   */
  using ArmSineTransitionFunctor =
      ArmSineTransitionFunctor_<LogicStateMachineT>;
  /**
  * @brief Action sequence that folds to right angle and aborts arm controller
  */
  using AbortArmControllerRightFold = base_functors::bActionSequence<
      boost::mpl::vector<AbortArmController, ArmRightFold>>;
  /**
  * @brief Action to take when starting folding arm before takeoff
  */
  using ArmPowerOnFold =
      base_functors::bActionSequence<boost::mpl::vector<ArmPowerOn, ArmFold>>;
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
