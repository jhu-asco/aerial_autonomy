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

/**
* @brief Transition action for tracking spiral reference for Airm system
*
* @tparam LogicStateMachineT logic state machine type
*/
template <class LogicStateMachineT>
struct MPCSpiralReferenceTrackingTransition
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief implementation logic for the transition action
  *
  * @param robot_system robot system
  */
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

/**
* @brief Transition action for tracking waypoint reference for Airm system
*
* @tparam LogicStateMachineT logic state machine type
*/
template <class LogicStateMachineT>
struct MPCWaypointReferenceTrackingTransition
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief shared pointer to waypoint reference for Airm system
  */
  using WaypointReferenceTrajectoryPtr =
      std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>;
  /**
  * @brief implementation logic for transition action
  *
  * @param robot_system robot system
  */
  void run(UAVArmSystem &robot_system) {
    VLOG(1) << "Setting waypoint reference goal for mpc controller";
    PositionYaw goal_position_yaw = conversions::protoPositionYawToPositionYaw(
        this->state_machine_config_.mpc_state_machine_config()
            .waypoint_reference());
    double goal_joint1 = this->state_machine_config_.mpc_state_machine_config()
                             .goal_joint_angle_1();
    double goal_joint2 = this->state_machine_config_.mpc_state_machine_config()
                             .goal_joint_angle_2();
    auto data = robot_system.getPose();
    tf::Vector3 current_position = data.getOrigin();
    double roll, pitch, yaw;
    data.getBasis().getEulerYPR(yaw, pitch, roll);
    // Add current position to goal (assuming goal is defined in local gravity
    // aligned coordinates
    goal_position_yaw = goal_position_yaw +
                        PositionYaw(current_position.x(), current_position.y(),
                                    current_position.z(), yaw);
    WaypointReferenceTrajectoryPtr reference = conversions::createWayPoint(
        goal_position_yaw, goal_joint1, goal_joint2);
    robot_system.setGoal<MPCControllerAirmConnector,
                         WaypointReferenceTrajectoryPtr>(reference);
  }
};

/**
 * @brief internal action while performing MPC control
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
/**
* @brief State that uses MPC control functor to track a reference trajectory
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using MPCState_ =
    BaseState<UAVSystem, LogicStateMachineT,
              MPCControlInternalActionFunctor_<LogicStateMachineT>>;
