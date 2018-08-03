#pragma once
/// States and actions corresponding to SYSID for UAV and arm
#include <aerial_autonomy/actions_guards/arm_states_actions.h>
#include <aerial_autonomy/actions_guards/uav_arm_functors.h>
#include <aerial_autonomy/actions_guards/uav_states_actions.h>
#include <boost/msm/front/functor_row.hpp>

/**
* @brief Class to provide typedefs for all basic uav states and actions
*
* @tparam LogicStateMachineT The backend of logic state machine
*/
template <class LogicStateMachineT>
struct UAVArmSysIDStatesActions : UAVStatesActions<LogicStateMachineT>,
                                  ArmStatesActions<LogicStateMachineT> {
  /**
   * @brief namespace for states and actions for basic uav actions
   */
  using usa = UAVStatesActions<LogicStateMachineT>;
  /**
   * @brief namespace for states and actions for arm functors
   */
  using asa = ArmStatesActions<LogicStateMachineT>;
  // States
  /**
  * @brief State during joystick and arm control
  */
  using RunningJoystickRPYTArmSineController =
      RunningJoystickRPYTArmSineController_<LogicStateMachineT>;

  /**
  * @brief State during arm control
  */
  using RunningArmSineController =
      RunningArmSineController_<LogicStateMachineT>;

  // Transition Actions
  /**
  * @brief Action sequence to abort arm controllers and move arm to
  * right angle
  */
  using AbortArmControllerArmRightFold =
      base_functors::bActionSequence<boost::mpl::vector<
          typename asa::AbortArmController, typename asa::ArmRightFold>>;
  /**
  * @brief Action sequence to abort UAV and arm controllers and move arm to
  * right angle
  */
  using AbortUAVArmControllerArmRightFold =
      base_functors::bActionSequence<boost::mpl::vector<
          typename usa::UAVControllerAbort, AbortArmControllerArmRightFold>>;
};
