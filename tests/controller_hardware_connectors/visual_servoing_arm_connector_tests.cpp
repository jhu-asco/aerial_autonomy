#include <gtest/gtest.h>

#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_arm_connector.h>
#include <aerial_autonomy/controllers/relative_pose_controller.h>
#include <aerial_autonomy/trackers/simple_tracker.h>

#include <arm_parsers/arm_simulator.h>
#include <quad_simulator_parser/quad_simulator.h>

#include <chrono>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

class VisualServoingControllerArmConnectorTests : public ::testing::Test {
public:
  VisualServoingControllerArmConnectorTests() : goal_tolerance_position(0.01) {
    PoseControllerConfig controller_config;
    controller_config.set_goal_position_tolerance(goal_tolerance_position);
    tf::Transform camera_transform = tf::Transform::getIdentity();
    camera_transform.setBasis(
        tf::Matrix3x3(0, 0, 1, -1, 0, 0, 0, -1, 0)); // Front facing camera
    tf::Transform arm_transform = tf::Transform::getIdentity();
    arm_transform.setBasis(
        tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1)); // Upside-down arm
    simple_tracker_.reset(new SimpleTracker(drone_hardware_, camera_transform));
    controller_.reset(new RelativePoseController(controller_config));
    visual_servoing_connector_.reset(new VisualServoingControllerArmConnector(
        *simple_tracker_, drone_hardware_, arm_hardware_, *controller_,
        camera_transform, arm_transform));
  }
  ArmSimulator arm_hardware_;
  QuadSimulator drone_hardware_;
  std::unique_ptr<RelativePoseController> controller_;
  std::unique_ptr<SimpleTracker> simple_tracker_;
  std::unique_ptr<VisualServoingControllerArmConnector>
      visual_servoing_connector_;
  double goal_tolerance_position;
};

TEST_F(VisualServoingControllerArmConnectorTests, Constructor) {}

TEST_F(VisualServoingControllerArmConnectorTests, CriticalRun) {
  // make tracking invalid:
  simple_tracker_->setTrackingIsValid(false);
  // Run connector
  visual_servoing_connector_->run();
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Critical);
}

TEST_F(VisualServoingControllerArmConnectorTests, GetTrackingPoseArmFrame) {
  // set tracking goal
  Position roi_goal(2, 0, 0.5);
  simple_tracker_->setTargetPositionGlobalFrame(roi_goal);
  // Get goal
  tf::Transform tracking_pose;
  tf::Transform expected_tracking_pose(tf::createQuaternionFromRPY(M_PI, 0, 0),
                                       tf::Vector3(2, 0, -0.5));
  ASSERT_TRUE(
      visual_servoing_connector_->getTrackingPoseArmFrame(tracking_pose));
  ASSERT_EQ(tracking_pose.getOrigin(), expected_tracking_pose.getOrigin());
  ASSERT_NEAR(tracking_pose.getRotation().angleShortestPath(
                  expected_tracking_pose.getRotation()),
              0., 1e-8);
}

TEST_F(VisualServoingControllerArmConnectorTests, SetGoal) {
  // set tracking goal
  Position roi_goal(2, 0, 0.5);
  simple_tracker_->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware_.setBatteryPercent(60);
  drone_hardware_.takeoff();
  // Enable arm
  arm_hardware_.sendCmd(ArmParser::POWER_ON);
  // Set goal
  tf::Transform relative_pose(tf::Quaternion(0, 0, 0, 1),
                              tf::Vector3(-0.5, 0, 0.1));
  visual_servoing_connector_->setGoal(relative_pose);
  // Run controller 100 times
  while (visual_servoing_connector_->getStatus() == ControllerStatus::Active) {
    visual_servoing_connector_->run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  Eigen::Matrix4d pose_eig = arm_hardware_.getEndEffectorTransform();
  tf::Transform arm_pose;
  conversions::transformMatrix4dToTf(pose_eig, arm_pose);
  ASSERT_NEAR(arm_pose.getOrigin().x(), 1.5, goal_tolerance_position);
  ASSERT_NEAR(arm_pose.getOrigin().y(), 0.0, goal_tolerance_position);
  ASSERT_NEAR(arm_pose.getOrigin().z(), -0.1, goal_tolerance_position);
  ASSERT_NEAR(arm_pose.getRotation().angleShortestPath(
                  tf::createQuaternionFromRPY(M_PI, 0, 0)),
              0., 1e-8);
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Completed);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
