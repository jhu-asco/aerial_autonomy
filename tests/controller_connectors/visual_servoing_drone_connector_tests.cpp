#include <gtest/gtest.h>

#include <aerial_autonomy/controller_connectors/visual_servoing_controller_drone_connector.h>
#include <aerial_autonomy/controllers/constant_heading_depth_controller.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <chrono>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

class VisualServoingControllerDroneConnectorTests : public ::testing::Test {
public:
  VisualServoingControllerDroneConnectorTests() : goal_tolerance_position(0.1) {
    ConstantHeadingDepthControllerConfig depth_config;
    depth_config.set_radial_gain(2.0);
    auto position_tolerance = depth_config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(goal_tolerance_position);
    position_tolerance->set_y(goal_tolerance_position);
    position_tolerance->set_z(goal_tolerance_position);
    tf::Transform camera_transform = tf::Transform::getIdentity();
    simple_tracker_.reset(new SimpleTracker(drone_hardware_, camera_transform));
    controller_.reset(new ConstantHeadingDepthController(depth_config));
    visual_servoing_connector_.reset(new VisualServoingControllerDroneConnector(
        *simple_tracker_, drone_hardware_, *controller_, camera_transform));
  }
  QuadSimulator drone_hardware_;
  std::unique_ptr<ConstantHeadingDepthController> controller_;
  std::unique_ptr<SimpleTracker> simple_tracker_;
  std::unique_ptr<VisualServoingControllerDroneConnector>
      visual_servoing_connector_;
  double goal_tolerance_position;
};

TEST_F(VisualServoingControllerDroneConnectorTests, Constructor) {}

TEST_F(VisualServoingControllerDroneConnectorTests, CriticalRun) {
  visual_servoing_connector_->setGoal(Position());
  // make tracking invalid:
  simple_tracker_->setTrackingIsValid(false);
  // Run connector
  visual_servoing_connector_->run();
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Critical);
}

TEST_F(VisualServoingControllerDroneConnectorTests, GetROIGlobalFrame) {
  // set tracking goal
  Position roi_goal(2, 0, 0.5);
  simple_tracker_->setTargetPositionGlobalFrame(roi_goal);
  // Get goal
  Position tracking_vector;
  ASSERT_TRUE(visual_servoing_connector_->getTrackingVectorGlobalFrame(
      tracking_vector));
  ASSERT_EQ(tracking_vector, roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware_.setBatteryPercent(60);
  drone_hardware_.takeoff();
}

TEST_F(VisualServoingControllerDroneConnectorTests, SetGoal) {
  // set tracking goal
  Position roi_goal(2, 0, 0.5);
  simple_tracker_->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware_.setBatteryPercent(60);
  drone_hardware_.takeoff();
  // set goal:
  Position object_position_camera(1, 0, 0);
  visual_servoing_connector_->setGoal(object_position_camera);
  // Run controller 100 times
  while (visual_servoing_connector_->getStatus() == ControllerStatus::Active) {
    visual_servoing_connector_->run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  parsernode::common::quaddata sensor_data;
  drone_hardware_.getquaddata(sensor_data);
  ASSERT_NEAR(sensor_data.localpos.x, roi_goal.x - object_position_camera.x,
              goal_tolerance_position);
  ASSERT_NEAR(sensor_data.localpos.y, roi_goal.y - object_position_camera.y,
              goal_tolerance_position);
  ASSERT_NEAR(sensor_data.localpos.z, roi_goal.z - object_position_camera.z,
              goal_tolerance_position);
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Completed);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
