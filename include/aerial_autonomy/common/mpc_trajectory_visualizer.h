#pragma once
#include "mpc_visualizer_config.pb.h"
#include <Eigen/Dense>
#include <aerial_autonomy/controller_connectors/mpc_controller_connector.h>
#include <gcop_comm/gcop_trajectory_visualizer.h>

/**
* @brief Helper class to visualize MPC trajectories using gcop comm package
*/
class MPCTrajectoryVisualizer {
  /**
  * @brief Namespace for GCOP MPC controller connector
  */
  using ControllerConnector =
      MPCControllerConnector<Eigen::VectorXd, Eigen::VectorXd>;

public:
  /**
  * @brief Constructor
  *
  * @param connector mpc controller connector
  * @param config visualizer specific config
  */
  MPCTrajectoryVisualizer(ControllerConnector &connector,
                          MPCVisualizerConfig config);
  /**
  * @brief publish trajectory and desired trajectory to rviz and rostopic
  */
  void publishTrajectory();
  /**
  * @brief Convert a vector of states into gcop comm trajectory
  *
  * @param xs vector of states
  * @param us vector of controls
  * @param skip_segments Number of segments to skip when filling trajectory
  *
  * @return filled gcop trajectory
  */
  gcop_comm::CtrlTraj getTrajectory(std::vector<Eigen::VectorXd> &xs,
                                    std::vector<Eigen::VectorXd> &us,
                                    int skip_segments);

  /**
  * @brief publish the current trajectory to rostopic
  */
  void publishGcopTrajectory();

private:
  ControllerConnector &connector_; ///< MPC Connector
  ros::NodeHandle nh_;             ///< Nodehandle to publish gcop trajectories
  ros::Publisher gcop_trajectory_pub_; ///< Publisher for gcop trajectories
  GcopTrajectoryVisualizer
      visualizer_; ///< Visualizer helper for publishing trajectories to Rviz
  MPCVisualizerConfig config_;       ///< Visualizer config
  std::vector<Eigen::VectorXd> xs_;  ///< Vector of states
  std::vector<Eigen::VectorXd> us_;  ///< Vector of controls
  std::vector<Eigen::VectorXd> xds_; ///< Vector of desired states
  std::vector<Eigen::VectorXd> uds_; ///< Vector of desired controls

private:
  /**
  * @brief Convert a single state vector to gcop comm state
  *
  * @param x state vector
  *
  * @return gcop comm state
  */
  gcop_comm::State getState(const Eigen::VectorXd &x);
};
