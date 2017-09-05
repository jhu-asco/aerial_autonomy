#pragma once
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/robot_systems/uav_sensor_system.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <aerial_autonomy/types/empty_goal.h>

/**
*
* @brief Internal functor while in joystick control state
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam AbortEventT Event to generate when aborting (Default
* ManualControlEvent)
*/
template <class LogicStateMachineT, class AbortEventT = ManualControlEvent>
struct JoystickControlInternalActionFunctor_
    : HoveringInternalActionFunctor_<LogicStateMachineT, AbortEventT> {};
/**
*
* @ brief Action while transitioning to joystick control
*
* @ tparam LogicStateMachineT logic state machine to process events
*/
template <class LogicStateMachineT>
struct JoystickControlTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSensorSystem, LogicStateMachineT> {
  void run(UAVSensorSystem &robot_system) {
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
    : EventAgnosticGuardFunctor<UAVSensorSystem, LogicStateMachineT> {
  bool guard(UAVSensorSystem &robot_system) {
    if (!sensor_status_to_bool(robot_system.getSensorStatus())) {
      LOG(WARNING) << "Sensor Status INVALID";
    }
    return sensor_status_to_bool(robot_system.getSensorStatus());
  }
};
/**
*@ brief Joystick control state that uses internal action
*
* @ tparam LogicStateMachineT logic state machine to process events
*/
template <class LogicStateMachineT>
using JoystickControlState_ =
    BaseState<UAVSensorSystem, LogicStateMachineT,
              JoystickControlInternalActionFunctor_<LogicStateMachineT>>;
