#include "aerial_autonomy/common/mpc_trajectory_visualizer.h"

MPCTrajectoryVisualizer::MPCTrajectoryVisualizer(MPCVisualizerConfig config)
    : nh_("mpc_visualizer"),
      visualizer_(nh_, config.parent_frame(), config.visualize_velocities()),
      config_(config) {
    VLOG(1) << "MPC Trajectory Visualizer Constructor";//TAGGED
  gcop_trajectory_pub_ =
      nh_.advertise<gcop_comm::CtrlTraj>("control_trajectory", 1);
}

void MPCTrajectoryVisualizer::publishTrajectory(
    ControllerConnector &connector) {
  ControllerStatus status = connector.getStatus();
  if (status == ControllerStatus::Active ||
      status == ControllerStatus::Completed) {
    connector.getTrajectory(xs_, us_);
    gcop_comm::CtrlTraj trajectory =
        getTrajectory(xs_, us_, config_.skip_segments());
    connector.getDesiredTrajectory(xds_, uds_);
    gcop_comm::CtrlTraj desired_trajectory =
        getTrajectory(xds_, uds_, config_.skip_segments());
    const auto &trajectory_color = config_.trajectory_color();
    visualizer_.setID(config_.trajectory_id());
    visualizer_.setColorLineStrip(trajectory_color.r(), trajectory_color.g(),
                                  trajectory_color.b(), trajectory_color.a());
    visualizer_.publishTrajectory(trajectory);
    const auto &desired_trajectory_color = config_.desired_trajectory_color();
    visualizer_.setID(config_.desired_trajectory_id());
    visualizer_.setColorLineStrip(
        desired_trajectory_color.r(), desired_trajectory_color.g(),
        desired_trajectory_color.b(), desired_trajectory_color.a());
    visualizer_.publishLineStrip(desired_trajectory);
    gcop_trajectory_pub_.publish(trajectory);
  }
}

void MPCTrajectoryVisualizer::publishGcopTrajectory(
    ControllerConnector &connector) {
  ControllerStatus status = connector.getStatus();
  if (status == ControllerStatus::Active ||
      status == ControllerStatus::Completed) {
    connector.getTrajectory(xs_, us_);
    gcop_comm::CtrlTraj trajectory =
        getTrajectory(xs_, us_, config_.skip_segments());
    gcop_trajectory_pub_.publish(trajectory);
  }
}
gcop_comm::State MPCTrajectoryVisualizer::getState(const Eigen::VectorXd &x) {
  gcop_comm::State state;
  if (x.size() < 15) {
    ROS_INFO("State size less than 15: %lu", x.size());
    return state;
  }
  state.basepose.translation.x = x[0];
  state.basepose.translation.y = x[1];
  state.basepose.translation.z = x[2];
  tf::Quaternion quat = tf::createQuaternionFromRPY(x[3], x[4], x[5]);
  tf::quaternionTFToMsg(quat, state.basepose.rotation);
  state.basetwist.linear.x = x[6];
  state.basetwist.linear.y = x[7];
  state.basetwist.linear.z = x[8];
  if (x.size() == 21) {
    // Joint angles, velocities
    for (int j = 0; j < 4; ++j) {
      state.statevector.push_back(x[15 + j]);
    }
  }
  return state;
}
gcop_comm::CtrlTraj
MPCTrajectoryVisualizer::getTrajectory(std::vector<Eigen::VectorXd> &xs,
                                       std::vector<Eigen::VectorXd> &us,
                                       int skip_segments) {
  gcop_comm::CtrlTraj control_trajectory;
  unsigned int N = us.size();
  for (unsigned int i = 0; i < N; i += skip_segments) {
    const auto &x = xs[i];
    control_trajectory.statemsg.push_back(getState(x));
    gcop_comm::Ctrl control;
    unsigned int us_size = us[i].size();
    control.ctrlvec.resize(us_size);
    control.ctrlvec[0] = us[i][0]; // thrust
    for (int j = 0; j < 2; ++j) {
      control.ctrlvec[j + 1] = x[12 + j];
    }
    control.ctrlvec[3] = us[i][3]; // yaw rate
    if (us_size == 6) {
      // Desired joint angles
      control.ctrlvec[4] = x[19];
      control.ctrlvec[5] = x[20];
    }
    control_trajectory.ctrl.push_back(control);
    // time
    control_trajectory.time.push_back(0.02 * i);
  }
  control_trajectory.statemsg.push_back(getState(xs.back()));
  control_trajectory.time.push_back(0.02 * N);
  control_trajectory.N = control_trajectory.ctrl.size();
  return control_trajectory;
}
