#include <gtest/gtest.h>

#include <aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_arm_connector.h>
#include <aerial_autonomy/controllers/relative_position_controller.h>
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
  VisualServoingControllerArmConnectorTests() : goal_tolerance_position(0.5) {
    PositionControllerConfig controller_config;
    controller_config.set_goal_position_tolerance(goal_tolerance_position);
    tf::Transform camera_transform = tf::Transform::getIdentity();
    camera_transform.setBasis(
        tf::Matrix3x3(0, 0, 1, -1, 0, 0, 0, -1, 0)); // Front facing camera
    tf::Transform arm_transform = tf::Transform::getIdentity();
    arm_transform.setBasis(
        tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1)); // Upside-down arm
    simple_tracker_.reset(new SimpleTracker(drone_hardware_, camera_transform));
    controller_.reset(new RelativePositionController(controller_config));
    visual_servoing_connector_.reset(new VisualServoingControllerArmConnector(
        *simple_tracker_, arm_hardware_, *controller_, camera_transform,
        arm_transform));
  }
  ArmSimulator arm_hardware_;
  QuadSimulator drone_hardware_;
  std::unique_ptr<RelativePositionController> controller_;
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

TEST_F(VisualServoingControllerArmConnectorTests, GetTrackingVectorArmFrame) {
  // set tracking goal
  Position roi_goal(2, 0, 0.5);
  simple_tracker_->setTargetPositionGlobalFrame(roi_goal);
  // Get goal
  Position tracking_vector;
  Position expected_tracking_vector(2, 0, -0.5);
  ASSERT_TRUE(
      visual_servoing_connector_->getTrackingVectorArmFrame(tracking_vector));
  ASSERT_EQ(tracking_vector, expected_tracking_vector);
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
  Position relative_position(-0.5, 0, 0.1);
  visual_servoing_connector_->setGoal(relative_position);
  // Run controller 100 times
  while (visual_servoing_connector_->getStatus() == ControllerStatus::Active) {
    visual_servoing_connector_->run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  Eigen::Matrix4d pose = arm_hardware_.getEndEffectorTransform();
  Position arm_position(pose(0, 3), pose(1, 3), pose(2, 3));
  ASSERT_NEAR(arm_position.x, 1.5, goal_tolerance_position);
  ASSERT_NEAR(arm_position.y, 0, goal_tolerance_position);
  ASSERT_NEAR(arm_position.z, 0.1, goal_tolerance_position);
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Completed);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
