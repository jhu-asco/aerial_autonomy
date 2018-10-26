#pragma once
#include "aerial_autonomy/common/controller_status.h"
#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"
#include "mpc_visualizer_config.pb.h"
#include <Eigen/Dense>
#include <chrono>
#include <gcop_comm/gcop_trajectory_visualizer.h>
#include <visualization_msgs/Marker.h>

/**
* @brief Helper class to visualize Qrotor backstepping control trajectories
* using gcop comm package
*/
class QrotorBacksteppingTrajectoryVisualizer {
public:
  /**
  * @brief Constructor
  *
  * @param connector qrotor backstepping controller connector
  * @param config visualizer specific config
  */
  QrotorBacksteppingTrajectoryVisualizer(
      MPCVisualizerConfig config,
      QrotorBacksteppingControllerConnector &connector)
      : nh_("qrotor_backstepping_trajectory_visualizer"), config_(config),
        visualizer_(nh_, config_.parent_frame(),
                    config_.visualize_velocities()),
        // points_pub_(nh_.advertise<visualization_msgs::Marker>(
        // "current_des_waypts", 5)),
        connector_(connector), goal_(connector.getGoal()),
        t0_(std::chrono::high_resolution_clock::now()) {
    const auto &desired_trajectory_color = config_.desired_trajectory_color();
    visualizer_.setID(config_.desired_trajectory_id());
    visualizer_.setColorLineStrip(
        desired_trajectory_color.r(), desired_trajectory_color.g(),
        desired_trajectory_color.b(), desired_trajectory_color.a());
  }
  /**
  * @brief publish desired trajectory to rviz and rostopic
  */
  void publishTrajectory(bool retain_marker) {
    goal_ = connector_.getGoal();
    if (!goal_) { // this is the NULL check
      std::cout << "goal is null!" << '\n';
      return;
    } else if (goal_) {
      ParticleState initial_pos = std::get<0>(goal_->atTime(0));
      std::cout << "initial: [ " << initial_pos.p.x << ", " << initial_pos.p.y
                << ", " << initial_pos.p.z << " ]" << '\n';
      ParticleState goal_pos = goal_->goal(0);
      std::cout << "goal: [ " << goal_pos.p.x << ", " << goal_pos.p.y << ", "
                << goal_pos.p.z << " ]" << '\n';
    }
    gcop_comm::CtrlTraj desired_trajectory = getTrajectory();
    // visualization_msgs::Marker way_points = getWayPoints(tau_vec_);
    // visualization_msgs::Marker current_desired_state =
    // getCurrentDesiredState();
    // current_desired_state.lifetime = ros::Duration();

    visualizer_.publishLineStrip(desired_trajectory, retain_marker);
    // visualizer_.publishVelocities(desired_trajectory);
    // points_pub_.publish(way_points);
    // points_pub_.publish(current_desired_state);
    // }
  }
  /**
  * @brief Convert a single state vector to gcop comm state
  *
  * @param x state vector
  *
  * @return gcop comm state
  */
  gcop_comm::State getState(ParticleState &particle_state) {
    gcop_comm::State state;
    // Position
    state.basepose.translation.x = particle_state.p.x;
    state.basepose.translation.y = particle_state.p.y;
    state.basepose.translation.z = particle_state.p.z;
    // Velocity
    state.basetwist.linear.x = particle_state.v.x;
    state.basetwist.linear.y = particle_state.v.y;
    state.basetwist.linear.z = particle_state.v.z;
    return state;
  }
  /**
  * @brief Convert a vector of states into gcop comm trajectory
  *
  * @param goal reference trajectory
  * @param tau_vec vector of time interval
  *
  * @return filled gcop trajectory
  */
  // gcop_comm::CtrlTraj
  // getTrajectory(std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>>
  // goal,
  //               Eigen::VectorXd tau_vec) {
  //   gcop_comm::CtrlTraj control_trajectory;
  //   double total_time = tau_vec.sum();
  //   unsigned int N = 50;
  //   for (unsigned int i = 0; i < N; i++) {
  //     double t = total_time / static_cast<double>(N) *
  //     static_cast<double>(i);
  //     // time
  //     control_trajectory.time.push_back(t);
  //     ParticleState desired_state = std::get<0>(goal->atTime(t));
  //     control_trajectory.statemsg.push_back(getState(desired_state));
  //   }
  //   // End of the trajectory
  //   control_trajectory.time.push_back(total_time);
  //   ParticleState desired_state_end = std::get<0>(goal->atTime(total_time));
  //   control_trajectory.statemsg.push_back(getState(desired_state_end));
  //   control_trajectory.N = N;
  //   return control_trajectory;
  // }
  gcop_comm::CtrlTraj getTrajectory() {
    gcop_comm::CtrlTraj control_trajectory;
    unsigned int N = 0;
    bool completed = false;
    double current_time = 0.0;
    double ahead_time = 0.0;
    while (!completed) {
      control_trajectory.time.push_back(current_time);
      ParticleState desired_state = std::get<0>(goal_->atTime(current_time));
      control_trajectory.statemsg.push_back(getState(desired_state));
      current_time += 1.0;
      N++;
      ahead_time = current_time;
      ParticleState ahead_desired_state =
          std::get<0>(goal_->atTime(ahead_time));
      double x_diff = ahead_desired_state.p.x - desired_state.p.x;
      double y_diff = ahead_desired_state.p.y - desired_state.p.y;
      double z_diff = ahead_desired_state.p.z - desired_state.p.z;
      if (x_diff == 0 && y_diff == 0 && z_diff == 0) {
        completed = true;
      }
    }
    control_trajectory.N = N - 1;
    return control_trajectory;
  }
  /**
  * @brief Get way points of reference trajectory
  *
  * @param goal reference trajectory
  * @param tau_vec vector of time interval
  *
  * @return marker for way points
  */
  // visualization_msgs::Marker
  // getWayPoints(std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>>
  // goal,
  //              Eigen::VectorXd tau_vec) {
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = config_.parent_frame();
  //   marker.header.stamp = ros::Time::now();
  //   marker.ns = "way_points";
  //   marker.id = 1;
  //   marker.type = visualization_msgs::Marker::POINTS;
  //   marker.action = visualization_msgs::Marker::ADD;
  //   // Set the scale and color
  //   marker.scale.x = .15;
  //   marker.scale.y = .15;
  //   marker.color.g = 1.0f;
  //   marker.color.a = 0.5;
  //   for (int i = 0; i < tau_vec.size(); i++) {
  //     // Get way points
  //     ParticleState states =
  //         std::get<0>(goal->atTime(tau_vec.head(i + 1).sum()));
  //     // Set the pose of the marker
  //     geometry_msgs::Point point;
  //     point.x = states.p.x;
  //     point.y = states.p.y;
  //     point.z = states.p.z;
  //     marker.points.push_back(point);
  //   }
  //   return marker;
  // }

  /**
  * @brief Get current desired state of reference trajectory
  *
  * @param goal reference trajectory
  * @param tau_vec vector of time interval
  * @param t0 start time
  *
  * @return marker for current desired state
  */
  // visualization_msgs::Marker getCurrentDesiredState() {
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = config_.parent_frame();
  //   marker.header.stamp = ros::Time::now();
  //   marker.ns = "current_desired_state";
  //   marker.id = 1;
  //   marker.type = visualization_msgs::Marker::SPHERE;
  //   marker.action = visualization_msgs::Marker::ADD;
  //   // Current time
  //   auto current_t = std::chrono::high_resolution_clock::now();
  //   std::chrono::duration<double> current_T = current_t - t0_;
  //   // Current desired state
  //   ParticleState current_desired_state =
  //       std::get<0>(goal_->atTime(current_T.count()));
  //   // Set the pose of the marker
  //   marker.pose.position.x = current_desired_state.p.x;
  //   marker.pose.position.y = current_desired_state.p.y;
  //   marker.pose.position.z = current_desired_state.p.z;
  //   marker.pose.orientation.x = 0.0;
  //   marker.pose.orientation.y = 0.0;
  //   marker.pose.orientation.z = 0.0;
  //   marker.pose.orientation.w = 1.0;
  //   // Set the scale and color
  //   marker.scale.x = 0.2;
  //   marker.scale.y = 0.2;
  //   marker.scale.z = 0.2;
  //   marker.color.r = 1.0;
  //   marker.color.b = 1.0;
  //   marker.color.a = 0.8;
  //   marker.lifetime = ros::Duration();
  //   return marker;
  // }

private:
  ros::NodeHandle nh_;         ///< Nodehandle to publish gcop trajectories
  MPCVisualizerConfig config_; ///< Visualizer config
  GcopTrajectoryVisualizer
      visualizer_; ///< Visualizer helper for publishing trajectories to Rviz
  // ros::Publisher points_pub_; ///< Point publisher
  // stdQrotorBacksteppingControllerConnector connector_; ///>
  QrotorBacksteppingControllerConnector &connector_;
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal_;
  std::chrono::high_resolution_clock::time_point t0_;
  bool started_ = false;
};
