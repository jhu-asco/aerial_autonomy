#include <aerial_autonomy/common/mpc_trajectory_visualizer.h>
#include <aerial_autonomy/tests/ddp_airm_mpc_connector_tests.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class MPCTrajectoryVisualizerTests : public MPCControllerAirmConnectorTests {
public:
  MPCTrajectoryVisualizerTests() : MPCControllerAirmConnectorTests() {
    visualizer_.reset(
        new MPCTrajectoryVisualizer(*controller_connector_, visualizer_config));
    visualize_traj_sub_ =
        nh_.subscribe("/desired_traj", 2,
                      &MPCTrajectoryVisualizerTests::markerSubscribe, this);
    gcop_traj_sub_ = nh_.subscribe(
        "/mpc_visualizer/control_trajectory", 2,
        &MPCTrajectoryVisualizerTests::controlTrajectorySubscribe, this);
    ros::spinOnce();
  }

protected:
  void markerSubscribe(const visualization_msgs::Marker &message) {
    received_messages_.push_back(message);
  }
  void controlTrajectorySubscribe(const gcop_comm::CtrlTraj &message) {
    received_ctrl_traj_ = message;
  }
  void enableConnector(const PositionYaw &initial_state,
                       const PositionYaw &goal,
                       std::vector<double> &&goal_joint_angles,
                       std::vector<double> &&init_joint_angles = {0, 0}) {
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_state.x;
    init_position.y = initial_state.y;
    init_position.z = initial_state.z;
    drone_hardware_.cmdwaypoint(init_position, initial_state.yaw);
    arm_simulator_.setJointAngles(init_joint_angles);
    controller_connector_->setGoal(conversions::createWayPoint(
        goal, goal_joint_angles[0], goal_joint_angles[1]));
    controller_connector_->run();
  }

protected:
  std::unique_ptr<MPCTrajectoryVisualizer> visualizer_;
  ros::Subscriber gcop_traj_sub_;
  ros::Subscriber visualize_traj_sub_;
  MPCVisualizerConfig visualizer_config;
  ros::NodeHandle nh_;
  std::vector<visualization_msgs::Marker> received_messages_;
  gcop_comm::CtrlTraj received_ctrl_traj_;
};

TEST_F(MPCTrajectoryVisualizerTests, testNotEngagedControlTrajectory) {
  visualizer_->publishGcopTrajectory();
  auto checkControlTrajectory = [&]() {
    ros::spinOnce();
    return (controller_config_.ddp_config().n() == received_ctrl_traj_.N);
  };
  ASSERT_FALSE(test_utils::waitUntilTrue()(checkControlTrajectory,
                                           std::chrono::seconds(1),
                                           std::chrono::milliseconds(1)));
}

TEST_F(MPCTrajectoryVisualizerTests, testActiveControlTrajectory) {
  enableConnector(PositionYaw(0, 0, 0, 0), PositionYaw(1, 1, 1.5, 0.5),
                  {-0.8, 1.4}, {-0.6, 1.0});
  visualizer_->publishGcopTrajectory();
  auto checkControlTrajectory = [&]() {
    ros::spinOnce();
    return (received_ctrl_traj_.ctrl.size() == received_ctrl_traj_.N);
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(checkControlTrajectory,
                                          std::chrono::seconds(2),
                                          std::chrono::milliseconds(1)));
}

TEST_F(MPCTrajectoryVisualizerTests, testActiveVisualizeTrajectory) {
  enableConnector(PositionYaw(0, 0, 0, 0), PositionYaw(1, 1, 1.5, 0.5),
                  {-0.8, 1.4}, {-0.6, 1.0});
  visualizer_->publishTrajectory();
  auto checkVisualizeTrajectory = [&]() {
    ros::spinOnce();
    return (received_messages_.size() == 2);
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(checkVisualizeTrajectory,
                                          std::chrono::seconds(100)));
  // Check messages are correct
  auto visualization_message = received_messages_.at(0);
  ASSERT_STREQ(visualization_message.header.frame_id.c_str(), "/optitrak");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mpc_trajectory_visualizer");
  return RUN_ALL_TESTS();
}
