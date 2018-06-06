#pragma once
#include "aerial_autonomy/actions_guards/arm_functors.h"
#include "aerial_autonomy/actions_guards/base_functors.h"
#include "aerial_autonomy/actions_guards/hovering_functors.h"
#include "aerial_autonomy/actions_guards/shorting_action_sequence.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h"
#include "aerial_autonomy/robot_systems/uav_arm_system.h"
#include "aerial_autonomy/types/spiral_reference_trajectory.h"
#include "aerial_autonomy/types/waypoint.h"
#include <tf_conversions/tf_eigen.h>

template <class LogicStateMachineT>
struct MPCSpiralReferenceTrackingTransition
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system) {
    VLOG(1) << "Setting spiral reference goal for mpc controller";
    auto spiral_reference_config =
        this->state_machine_config_.mpc_state_machine_config()
            .spiral_reference();
    auto arm_reference_config =
        this->state_machine_config_.mpc_state_machine_config().arm_reference();
    auto data = robot_system.getPose();
    Eigen::Vector3d current_position;
    tf::vectorTFToEigen(data.getOrigin(), current_position);
    double r, p, y;
    data.getBasis().getEulerYPR(y, p, r);
    SpiralReferenceTrajectoryPtr reference(new SpiralReferenceTrajectory(
        spiral_reference_config, arm_reference_config, current_position, y));
    robot_system.setGoal<MPCControllerAirmConnector,
                         SpiralReferenceTrajectoryPtr>(reference);
  }
};

template <class LogicStateMachineT>
struct MPCWaypointReferenceTrackingTransition
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  using WaypointReferenceTrajectoryPtr =
      Waypoint<Eigen::VectorXd, Eigen::VectorXd>;
  void run(UAVArmSystem &robot_system) {
    VLOG(1) << "Setting waypoint reference goal for mpc controller";
    PositionYaw goal_positon_yaw = conversions::protoPositionYawToPositionYaw(
        this->state_machine_config_.mpc_state_machine_config()
            .waypoint_reference());
    double goal_joint1 = this->state_machine_config_.mpc_state_machine_config()
                             .goal_joint_angle_1();
    double goal_joint2 = this->state_machine_config_.mpc_state_machine_config()
                             .goal_joint_angle_2();
    auto data = robot_system.getPose();
    // Add current position to goal (assuming goal is defined in local gravity
    // aligned coordinates
    goal_position_yaw =
        goal_position_yaw + PositionYaw(data.localpos.x, data.localpos.y,
                                        data.localpos.z, data.rpydata.z);
    WaypointReferenceTrajectoryPtr reference(conversions::createWayPoint(
        goal_position_yaw, goal_joint1, goal_joint2));
    robot_system.setGoal<MPCControllerAirmConnector,
                         WaypointReferenceTrajectoryPtr>(reference);
  }
};

/**
 * @brief internal action while performing position control
 *
* @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT>
using MPCControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, MPCControllerAirmConnector, false>>>;
