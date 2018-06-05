#pragma once
#include "mpc_visualizer_config.pb.h"
#include <Eigen/Dense>
#include <aerial_autonomy/controller_connectors/mpc_controller_connector.h>
#include <gcop_comm/gcop_trajectory_visualizer.h>

class MPCTrajectoryVisualizer {
  using ControllerConnector =
      MPCControllerConnector<Eigen::VectorXd, Eigen::VectorXd>;

public:
  MPCTrajectoryVisualizer(ControllerConnector &connector,
                          MPCVisualizerConfig config);
  void publishTrajectory();
  gcop_comm::CtrlTraj getTrajectory(std::vector<Eigen::VectorXd> &xs,
                                    std::vector<Eigen::VectorXd> &us,
                                    int skip_segments);

  void publishGcopTrajectory();

private:
  ControllerConnector &connector_;
  ros::NodeHandle nh_;
  ros::Publisher gcop_trajectory_pub_;
  GcopTrajectoryVisualizer visualizer_;
  MPCVisualizerConfig config_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<Eigen::VectorXd> xds_;
  std::vector<Eigen::VectorXd> uds_;
};
