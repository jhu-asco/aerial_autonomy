#pragma once
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <aerial_autonomy/types/empty_goal.h>

/**
*
* @ brief Action while transitioning to joystick control
*
* @ tparam LogicStateMachineT logic state machine to process events
*/
template <class LogicStateMachineT>
struct JoystickControlTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    VLOG(1) << "entering joystick control mode";
    robot_system.setGoal<JoystickVelocityControllerDroneConnector, EmptyGoal>(
        EmptyGoal());
  }
};
/**
* @brief Guard to check validity of sensor data before switching to joystick
* control state
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct JoystickControlTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system) {
    if (!bool(robot_system.getVelocityPoseSensorStatus())) {
      LOG(WARNING) << "Sensor Status INVALID";
    }
    return bool(robot_system.getVelocityPoseSensorStatus());
  }
};
/**
 * @brief internal action while performing joystick control
 *
* @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT>
using JoystickControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        HoveringInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, JoystickVelocityControllerDroneConnector>>>;

/**
*@ brief Joystick control state that uses internal action
*
* @ tparam LogicStateMachineT logic state machine to process events
*/
template <class LogicStateMachineT>
using JoystickControlState_ =
    BaseState<UAVSystem, LogicStateMachineT,
              JoystickControlInternalActionFunctor_<LogicStateMachineT>>;
