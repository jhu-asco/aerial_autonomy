#pragma once
#include "mpc_visualizer_config.pb.h"
#include <Eigen/Dense>
#include <aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h>
#include <chrono>
#include <gcop_comm/gcop_trajectory_visualizer.h>
#include <visualization_msgs/Marker.h>

/**
* @brief Helper class to visualize Qrotor backstepping control trajectories
* using
* gcop comm package
*/
class QrotorBacksteppingTrajectoryVisualizer {
public:
  /**
  * @brief Constructor
  *
  * @param connector qrotor backstepping controller connector
  * @param config visualizer specific config
  */
  QrotorBacksteppingTrajectoryVisualizer(MPCVisualizerConfig config)
      : nh_("qrotor_backstepping_trajectory_visualizer"), config_(config),
        visualizer_(nh_, config_.parent_frame(),
                    config_.visualize_velocities()) {
    way_points_pub_ =
        nh_.advertise<visualization_msgs::Marker>("way_point_marker", 1);
    current_des_pub_ =
        nh_.advertise<visualization_msgs::Marker>("current_des_marker", 1);
  }
  /**
  * @brief publish desired trajectory to rviz and rostopic
  */
  void publishTrajectory(
      std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
      Eigen::VectorXd tau_vec,
      std::chrono::high_resolution_clock::time_point t0) {
    gcop_comm::CtrlTraj desired_trajectory = getTrajectory(goal, tau_vec);
    visualization_msgs::Marker way_points = getWayPoints(goal, tau_vec);
    visualization_msgs::Marker current_desired_state =
        getCurrentDesiredState(goal, t0);
    const auto &desired_trajectory_color = config_.desired_trajectory_color();
    visualizer_.setID(config_.desired_trajectory_id());
    visualizer_.setColorLineStrip(
        desired_trajectory_color.r(), desired_trajectory_color.g(),
        desired_trajectory_color.b(), desired_trajectory_color.a());
    visualizer_.publishLineStrip(desired_trajectory);
    visualizer_.publishVelocities(desired_trajectory);
    way_points_pub_.publish(way_points);
    current_des_pub_.publish(current_desired_state);
  }
  /**
  * @brief Convert a single state vector to gcop comm state
  *
  * @param x state vector
  *
  * @return gcop comm state
  */
  gcop_comm::State getState(ParticleState &x) {
    gcop_comm::State state;
    // Position
    state.basepose.translation.x = x.p.x;
    state.basepose.translation.y = x.p.y;
    state.basepose.translation.z = x.p.z;
    // Velocity
    state.basetwist.linear.x = x.v.x;
    state.basetwist.linear.y = x.v.y;
    state.basetwist.linear.z = x.v.z;
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
  gcop_comm::CtrlTraj
  getTrajectory(std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
                Eigen::VectorXd tau_vec) {
    gcop_comm::CtrlTraj control_trajectory;
    double total_time = tau_vec.sum();
    unsigned int N = 50;
    for (unsigned int i = 0; i < N; i++) {
      double t = total_time / static_cast<double>(N) * static_cast<double>(i);
      // time
      control_trajectory.time.push_back(t);
      ParticleState desired_state = std::get<0>(goal->atTime(t));
      control_trajectory.statemsg.push_back(getState(desired_state));
    }
    // End of the trajectory
    control_trajectory.time.push_back(total_time);
    ParticleState desired_state_end = std::get<0>(goal->atTime(total_time));
    control_trajectory.statemsg.push_back(getState(desired_state_end));
    control_trajectory.N = N;
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
  visualization_msgs::Marker
  getWayPoints(std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
               Eigen::VectorXd tau_vec) {
    visualization_msgs::Marker points;
    points.header.frame_id = "/optitrak";
    points.header.stamp = ros::Time::now();
    points.ns = "way_points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = .15;
    points.scale.y = .15;
    points.color.g = 1.0f;
    points.color.a = 0.5;
    for (int i = 0; i < tau_vec.size(); i++) {
      ParticleState states =
          std::get<0>(goal->atTime(tau_vec.head(i + 1).sum()));
      geometry_msgs::Point p;
      p.x = states.p.x;
      p.y = states.p.y;
      p.z = states.p.z;
      points.points.push_back(p);
    }
    return points;
  }

  /**
  * @brief Get current desired state of reference trajectory
  *
  * @param goal reference trajectory
  * @param tau_vec vector of time interval
  * @param t0 start time
  *
  * @return marker for current desired state
  */
  visualization_msgs::Marker getCurrentDesiredState(
      std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
      std::chrono::high_resolution_clock::time_point t0) {
    visualization_msgs::Marker points;
    points.header.frame_id = "/optitrak";
    points.header.stamp = ros::Time::now();
    points.ns = "current_desired_state";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = .2;
    points.scale.y = .2;
    points.color.r = 1.0;
    points.color.b = 1.0;
    points.color.a = 0.8;
    auto current_t = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> current_T = current_t - t0;
    ParticleState current_desired_state =
        std::get<0>(goal->atTime(current_T.count()));
    geometry_msgs::Point p;
    p.x = current_desired_state.p.x;
    p.y = current_desired_state.p.y;
    p.z = current_desired_state.p.z;
    points.points.push_back(p);
    return points;
  }

private:
  ros::NodeHandle nh_;         ///< Nodehandle to publish gcop trajectories
  MPCVisualizerConfig config_; ///< Visualizer config
  GcopTrajectoryVisualizer
      visualizer_; ///< Visualizer helper for publishing trajectories to Rviz
  ros::Publisher way_points_pub_;  ///< Publisher for way points
  ros::Publisher current_des_pub_; ///< Publisher for current desired state
};
