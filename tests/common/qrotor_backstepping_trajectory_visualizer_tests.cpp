#include "aerial_autonomy/common/qrotor_backstepping_trajectory_visualizer.h"
#include "aerial_autonomy/common/proto_utils.h"
#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"
#include "aerial_autonomy/tests/qrotor_backstepping_controller_connector_tests.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "minimum_snap_reference_trajectory_config.pb.h"
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>

class QrotorBacksteppingTrajectoryVisualizerTests
    : public QrotorBacksteppingControllerConnectorTests {
public:
  QrotorBacksteppingTrajectoryVisualizerTests()
      : QrotorBacksteppingControllerConnectorTests() {
    QrotorBacksteppingControllerConfig config;
    if (!proto_utils::loadProtoText(
            std::string(PROJECT_SOURCE_DIR) +
                "/param/qrotor_backstepping_controller_config.pbtxt",
            config)) {
      LOG(ERROR) << "Cannot load proto file for the controller";
    }
    quad_simulator::QuadSimulator drone_hardware;
    ThrustGainEstimator thrust_gain_estimator(0.16);
    QrotorBacksteppingController controller(config);
    QrotorBacksteppingControllerConnector controller_connector(
        drone_hardware, controller, thrust_gain_estimator, config);
    MinimumSnapReferenceTrajectoryConfig ref_config;
    if (!proto_utils::loadProtoText(
            std::string(PROJECT_SOURCE_DIR) +
                "/param/minimum_snap_reference_trajectory_config.pbtxt",
            ref_config)) {
      LOG(ERROR) << "Cannot load proto file for the reference trajectory";
    }
    std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
        new MinimumSnapReferenceTrajectory(ref_config));
    controller_connector.setGoal(goal);
    visualizer_.reset(new QrotorBacksteppingTrajectoryVisualizer(
        visualizer_config, controller_connector));
    visualize_traj_sub_ = nh_.subscribe(
        "/desired_traj", 2,
        &QrotorBacksteppingTrajectoryVisualizerTests::markerSubscribe, this);
    ros::spinOnce();
  }

protected:
  void markerSubscribe(const visualization_msgs::Marker &message) {
    received_messages_.push_back(message);
  }

  std::unique_ptr<QrotorBacksteppingTrajectoryVisualizer> visualizer_;
  ros::Subscriber visualize_traj_sub_;
  MPCVisualizerConfig visualizer_config;
  ros::NodeHandle nh_;
  std::vector<visualization_msgs::Marker> received_messages_;
};

TEST_F(QrotorBacksteppingTrajectoryVisualizerTests,
       testVisualizeReferenceTrajectory) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 10;
  Eigen::MatrixXd path(2, 3);
  path << 0, 0, 0, 1, 1, 1;
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  // auto t0 = std::chrono::high_resolution_clock::now();
  visualizer_->publishTrajectory(true);
  auto checkVisualizeTrajectory = [&]() {
    ros::spinOnce();
    return (received_messages_.size() == 1);
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(checkVisualizeTrajectory,
                                          std::chrono::seconds(60)));

  // Check messages are correct
  auto visualization_message = received_messages_.at(0);
  ASSERT_STREQ(visualization_message.header.frame_id.c_str(), "/optitrak");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "qrotor_backstepping_trajectory_visualizer");
  return RUN_ALL_TESTS();
}
