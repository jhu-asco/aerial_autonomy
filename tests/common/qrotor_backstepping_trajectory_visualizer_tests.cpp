#include <aerial_autonomy/common/qrotor_backstepping_trajectory_visualizer.h>
#include <aerial_autonomy/tests/qrotor_backstepping_controller_connector_tests.h>
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>

class QrotorBacksteppingTrajectoryVisualizerTests
    : public QrotorBacksteppingControllerConnectorTests {
public:
  QrotorBacksteppingTrajectoryVisualizerTests()
      : QrotorBacksteppingControllerConnectorTests() {
    visualizer_.reset(
        new QrotorBacksteppingTrajectoryVisualizer(visualizer_config));
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
  auto t0 = std::chrono::high_resolution_clock::now();
  visualizer_->publishTrajectory(goal, tau_vec, t0);
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
