#pragma once
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>

/**
* @brief Logic to check arm power and manual mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ManualControlArmInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ManualControlInternalActionFunctor_<LogicStateMachineT>>>;

/**
* @brief Logic to check uav status, arm status, arm sine controller status
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ArmSineControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, ArmSineControllerConnector, false>>>;

/**
* @brief Logic to check uav status, arm status, rpyt, arm sine controller status
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using JoystickRPYTArmSineControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        JoystickRPYTControlInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, ArmSineControllerConnector, false>>>;

/**
* @brief State that command rpyt messages based on joystick and moves arm
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using RunningJoystickRPYTArmSineController_ = BaseState<
    UAVArmSystem, LogicStateMachineT,
    JoystickRPYTArmSineControlInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State that command moves arm
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using RunningArmSineController_ =
    BaseState<UAVArmSystem, LogicStateMachineT,
              ArmSineControlInternalActionFunctor_<LogicStateMachineT>>;
