
#include <gtest/gtest.h>

#include <aerial_autonomy/controller_hardware_connectors/relative_pose_visual_servoing_controller_drone_connector.h>
#include <aerial_autonomy/controllers/velocity_based_relative_pose_controller.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <chrono>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

using namespace test_utils;

class RelativePoseVisualServoingControllerDroneConnectorTests
    : public ::testing::Test {
public:
  RelativePoseVisualServoingControllerDroneConnectorTests()
      : goal_tolerance_position_(0.01), goal_tolerance_yaw_(0.01) {
    VelocityBasedRelativePoseControllerConfig config;
    auto position_controller_config =
        config.mutable_velocity_based_position_controller_config();
    position_controller_config->set_position_gain(2.0);
    position_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(goal_tolerance_yaw_);
    auto position_tolerance =
        position_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    position_tolerance->set_x(goal_tolerance_position_);
    position_tolerance->set_y(goal_tolerance_position_);
    position_tolerance->set_z(goal_tolerance_position_);
    tf::Transform camera_transform = tf::Transform::getIdentity();
    simple_tracker_.reset(new SimpleTracker(drone_hardware_, camera_transform));
    controller_.reset(new VelocityBasedRelativePoseController(config));
    visual_servoing_connector_.reset(
        new RelativePoseVisualServoingControllerDroneConnector(
            *simple_tracker_, drone_hardware_, *controller_, camera_transform));
  }
  QuadSimulator drone_hardware_;
  std::unique_ptr<VelocityBasedRelativePoseController> controller_;
  std::unique_ptr<SimpleTracker> simple_tracker_;
  std::unique_ptr<RelativePoseVisualServoingControllerDroneConnector>
      visual_servoing_connector_;
  double goal_tolerance_position_;
  double goal_tolerance_yaw_;
};

TEST_F(RelativePoseVisualServoingControllerDroneConnectorTests, Constructor) {}

TEST_F(RelativePoseVisualServoingControllerDroneConnectorTests, CriticalRun) {
  // make tracking invalid:
  simple_tracker_->setTrackingIsValid(false);
  // Run connector
  visual_servoing_connector_->run();
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Critical);
}

TEST_F(RelativePoseVisualServoingControllerDroneConnectorTests,
       GetTransformQuadFrame) {
  // set tracking vector
  tf::Transform goal(tf::Quaternion(), tf::Vector3(2, 0, 0.5));
  simple_tracker_->setTargetPoseGlobalFrame(goal);
  // Get vector
  tf::Transform tracking_vector;
  ASSERT_TRUE(
      visual_servoing_connector_
          ->getTrackingTransformRotationCompensatedQuadFrame(tracking_vector));
  ASSERT_TF_NEAR(tracking_vector, goal);
}

TEST_F(RelativePoseVisualServoingControllerDroneConnectorTests,
       RunUntilConvergence) {
  // set tracking goal
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, -0.1),
                             tf::Vector3(2, -0.5, 0.5));
  simple_tracker_->setTargetPoseGlobalFrame(tracked_pose);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware_.setBatteryPercent(60);
  drone_hardware_.takeoff();
  // Set goal
  tf::Transform goal_relative_pose(tf::createQuaternionFromRPY(0, 0, 0.5),
                                   tf::Vector3(1, 0, 0));
  visual_servoing_connector_->setGoal(goal_relative_pose);
  // Run controller until inactive
  while (visual_servoing_connector_->getStatus() == ControllerStatus::Active) {
    visual_servoing_connector_->run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  // Check position is the goal position
  parsernode::common::quaddata sensor_data;
  drone_hardware_.getquaddata(sensor_data);
  tf::Transform quad_transform(
      tf::createQuaternionFromRPY(sensor_data.rpydata.x, sensor_data.rpydata.y,
                                  sensor_data.rpydata.z),
      tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                  sensor_data.localpos.z));
  ASSERT_TF_NEAR(quad_transform, tracked_pose * goal_relative_pose,
                 goal_tolerance_yaw_);
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Completed);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
