#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <glog/logging.h>
#include <parsernode/common.h>
#include <aerial_autonomy/controller_hardware_connectors/rpyt_based_odom_sensor_controller_drone_connector.h>

namespace be = uav_basic_events;

// Goals for search pattern (x,y,z,yaw)
PositionYaw goal1(0.0,0.0,1.0,0);
PositionYaw goal2(0.0,1.0,1.0,0);
PositionYaw goal3(-0.5,1.0,1.0,0);
PositionYaw goal4(-0.5,0.0,1.0,0);


template <class LogicStateMachineT,class DroneConnectorT = RPYTBasedPositionControllerDroneConnector>
struct PositionControlTransitionActionGoal1Functor
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    robot_system
        .setGoal<DroneConnectorT, PositionYaw>(goal1);
  }
};
template <class LogicStateMachineT,class DroneConnectorT = RPYTBasedPositionControllerDroneConnector>
struct PositionControlTransitionActionGoal2Functor
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    robot_system
        .setGoal<DroneConnectorT, PositionYaw>(goal2);
  }
};
template <class LogicStateMachineT,class DroneConnectorT = RPYTBasedPositionControllerDroneConnector>
struct PositionControlTransitionActionGoal3Functor
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    robot_system
        .setGoal<DroneConnectorT, PositionYaw>(goal3);
  }
};
template <class LogicStateMachineT,class DroneConnectorT = RPYTBasedPositionControllerDroneConnector>
struct PositionControlTransitionActionGoal4Functor
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    robot_system
        .setGoal<DroneConnectorT, PositionYaw>(goal4);
  }
};
/**
* @brief Transition action to perform when going into position control mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT,class DroneConnectorT = RPYTBasedPositionControllerDroneConnector>
struct PositionControlTransitionActionFunctor_
    : ActionFunctor<PositionYaw, UAVSystem, LogicStateMachineT> {
  void run(const PositionYaw &goal, UAVSystem &robot_system) {
    robot_system
        .setGoal<DroneConnectorT, PositionYaw>(goal);
  }
};

/**
* @brief Transition action to perform when aborting any UAV Controller
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct UAVControllerAbortActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    LOG(WARNING) << "Aborting UAV Controller";
    robot_system.abortController(HardwareType::UAV);
  }
};

/**
* @brief Guard function to check the goal is within tolerance before starting
* towards goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PositionControlTransitionGuardFunctor_
    : GuardFunctor<PositionYaw, UAVSystem, LogicStateMachineT> {
  bool guard(const PositionYaw &goal, UAVSystem &robot_system) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    geometry_msgs::Vector3 current_position = data.localpos;
    // Check goal is close to position before sending goal (Can use a geofence
    // here)
    const double &tolerance_pos =
        robot_system.getConfiguration().max_goal_distance();
    bool result = true;
    if (std::abs(current_position.x - goal.x) > tolerance_pos ||
        std::abs(current_position.y - goal.y) > tolerance_pos ||
        std::abs(current_position.z - goal.z) > tolerance_pos) {
      LOG(WARNING) << "Goal not within the position tolerance";
      result = false;
    }
    return result;
  }
};

/**
 * @brief Check the velocity along xyz axes are above 0.05 m/s
 *
 * @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT>
struct ZeroVelocityGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    geometry_msgs::Vector3 current_vel = data.linvel;
    const double tolerance_vel = .05;
    bool result = true;
    if (std::abs(current_vel.x) > tolerance_vel ||
        std::abs(current_vel.y) > tolerance_vel ||
        std::abs(current_vel.z) > tolerance_vel) {
      LOG(WARNING) << "Velocity not within tolerance";
      result = false;
    }
    return result;
  }
};

/**
 * @brief internal action while performing position control
 *
* @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT, class DroneConnectorT>
using PositionControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, DroneConnectorT>>>;


template <class LogicStateMachineT>
using ReachingGoal_ =
    BaseState<UAVSystem, LogicStateMachineT,
              PositionControlInternalActionFunctor_<LogicStateMachineT,
                                                    RPYTBasedOdomSensorControllerDroneConnector>>;

