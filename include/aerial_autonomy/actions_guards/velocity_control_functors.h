#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/controller_hardware_connectors/builtin_velocity_controller_drone_connector.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <glog/logging.h>
#include <parsernode/common.h>

namespace be = uav_basic_events;

/**
* @brief Transition action to switch on builtin velocity controller
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VelocityControlTransitionActionFunctor_
    : ActionFunctor<VelocityYaw, UAVSystem, LogicStateMachineT> {
  void run(const VelocityYaw &goal, UAVSystem &robot_system) {
    robot_system.setGoal<BuiltInVelocityControllerDroneConnector, VelocityYaw>(
        goal);
  }
};

/**
* @brief Guard function to check the goal velocity is within tolerance before
* initializing the velocity controller
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VelocityControlTransitionGuardFunctor_
    : GuardFunctor<VelocityYaw, UAVSystem, LogicStateMachineT> {
  bool guard(const VelocityYaw &goal, UAVSystem &robot_system) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    const double &tolerance_vel =
        robot_system.getConfiguration().max_goal_velocity();
    bool result = true;
    if (std::abs(goal.x) > tolerance_vel || std::abs(goal.y) > tolerance_vel ||
        std::abs(goal.z) > tolerance_vel) {
      LOG(WARNING) << "Goal not within velocity tolerance alon x or y or z";
      result = false;
    }
    return result;
  }
};

/**
 * @brief internal action while performing velocity control. Only
 * checks for critical state and does not abort on completed
 *
* @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT>
using VelocityControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, BuiltInVelocityControllerDroneConnector,
            false>>>;

/**
* @brief State that uses velocity control functor to track commanded velocity
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ExecutingVelocityGoal_ =
    BaseState<UAVSystem, LogicStateMachineT,
              VelocityControlInternalActionFunctor_<LogicStateMachineT>>;
