#pragma once
#include "aerial_autonomy/common/proto_utils.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/controller_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/controller_connectors/rpyt_relative_pose_adaptive_estimate_connector.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/polynomial_reference_trajectory.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <glog/logging.h>
#include <parsernode/common.h>

#include <aerial_autonomy/types/circle_reference_trajectory.h>
#include <aerial_autonomy/types/hover_reference_trajectory.h>
#include <aerial_autonomy/types/minimum_snap_reference_trajectory_yaw.h>

namespace be = uav_basic_events;

/**
* @brief Transition action to perform when going into position control mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PositionControlTransitionActionFunctor_
    : ActionFunctor<PositionYaw, UAVSystem, LogicStateMachineT> {
  void run(const PositionYaw &goal, UAVSystem &robot_system) {
    tf::StampedTransform start_pose = robot_system.getPose();
    auto &reference_config =
        this->state_machine_config_.poly_reference_config();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> reference(
        new PolynomialReferenceTrajectory(goal, start_position_yaw,
                                          reference_config));
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            reference);
  }
};

/**
* @brief Transition action to perform when going into position control mode
* with the adaptive controller
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct AdaptiveTransitionActionFunctor_
    : ActionFunctor<PositionYaw, UAVSystem, LogicStateMachineT> {
  void run(const PositionYaw &goal, UAVSystem &robot_system) {
    tf::StampedTransform start_pose = robot_system.getPose();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    // Minimum snap reference trajectory config
    auto reference_config = MinimumSnapReferenceTrajectoryConfig();
    // Waypoints config
    auto waypoint_config =
        reference_config.mutable_following_waypoint_sequence_config();
    CHECK_EQ((waypoint_config->way_points()).size(), 0)
        << "Should not input any waypoints. They are automatically specified "
           "based on start and goal";
    // Add waypoints
    conversions::setWaypoint(waypoint_config->add_way_points(),
                             start_position_yaw);
    conversions::setWaypoint(waypoint_config->add_way_points(), goal);
    // L2 distance between start, goal
    double x = start_position_yaw.x - goal.x;
    double y = start_position_yaw.y - goal.y;
    double z = start_position_yaw.z - goal.z;
    double distance = std::sqrt((x * x) + (y * y) + (z * z));
    double average_velocity = reference_config.average_velocity();
    // minimum time interval
    double tau_min = 7.0;
    CHECK_EQ(reference_config.tau_vec().size(), 0)
        << "Should not input any time steps. They are automatically specified "
           "based on input average velocity";
    // Add time interval
    reference_config.add_tau_vec(
        std::max(distance / average_velocity, tau_min));
    // Minimum snap reference trajectory object
    std::shared_ptr<ReferenceTrajectory<ParticleStateYaw, Snap>> reference(
        new MinimumSnapReferenceTrajectoryYaw(reference_config,goal.yaw));
    robot_system.setGoal<
        RPYTRelativePoseAdaptiveEstimateConnector,
        ReferenceTrajectoryPtr<ParticleStateYaw, Snap>>(
        reference);
    // robot_system.visualizeTrajectory(reference); Removed until arguments can
    // be ironed out
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
    robot_system.abortController(ControllerGroup::UAV);
    LOG(WARNING) << "Aborting HighLevel Controller";
    robot_system.abortController(ControllerGroup::HighLevel);
    LOG(WARNING) << "Done aborting";
  }
};

/**
* @brief Action to set home location
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct SetHomeTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    VLOG(1) << "Selecting home location";
    robot_system.setHomeLocation();
  }
};

/**
* @brief Action to reach a pre designated point
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GoHomeTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    PositionYaw home_location = robot_system.getHomeLocation();
    VLOG(1) << "Going home";
    tf::StampedTransform start_pose = robot_system.getPose();
    auto &reference_config =
        this->state_machine_config_.poly_reference_config();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> reference(
        new PolynomialReferenceTrajectory(home_location, start_position_yaw,
                                          reference_config));
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            reference);
  }
};

/**
* @brief Guard for home transition
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GoHomeTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system) {
    LOG(INFO) << "Checking home location specified and pose sensor status";
    return robot_system.isHomeLocationSpecified() &&
           sensor_status_to_bool(robot_system.getPoseSensorStatus());
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
        std::abs(current_position.z - goal.z) > tolerance_pos ||
        !sensor_status_to_bool(robot_system.getPoseSensorStatus())) {
      LOG(WARNING)
          << "Goal not within the position tolerance or sensor invalid";
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
template <class LogicStateMachineT>
using PositionControlInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT,
            RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>>>>;

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ReachingGoal_ =
    BaseState<UAVSystem, LogicStateMachineT,
              PositionControlInternalActionFunctor_<LogicStateMachineT>>;
