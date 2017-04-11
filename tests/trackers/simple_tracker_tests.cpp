#include <gtest/gtest.h>

#include "aerial_autonomy/trackers/simple_tracker.h"

#include <quad_simulator_parser/quad_simulator.h>

#include <glog/logging.h>

using namespace quad_simulator;

TEST(SimpleTrackerTests, Constructor) {
  QuadSimulator drone_hardware;
  UAVVisionSystemConfig config;
  for (int i = 0; i < 6; ++i) {
    config.add_camera_transform(0.0);
  }
  config.set_desired_visual_servoing_distance(1.0);
  SimpleTracker simple_tracker(drone_hardware, config);
}

TEST(SimpleTrackerTests, InvalidCameraTransform) {
  QuadSimulator drone_hardware;
  UAVVisionSystemConfig config;
  SimpleTracker simple_tracker(drone_hardware, config);
  ASSERT_FALSE(simple_tracker.trackingIsValid());
}

TEST(SimpleTrackerTests, TrackingVector) {
  QuadSimulator drone_hardware;
  UAVVisionSystemConfig config;
  for (int i = 0; i < 6; ++i) {
    config.add_camera_transform(0.0);
  }
  SimpleTracker simple_tracker(drone_hardware, config);
  Position position;
  Position tracking_vector;
  // Test tracking is valid
  ASSERT_TRUE(simple_tracker.trackingIsValid());
  // Test getting tracking vector
  simple_tracker.getTrackingVector(position);
  ASSERT_EQ(position, Position(0, 0, 0));
  // Test setting tracking vector
  position = Position(1, 1, 1);
  simple_tracker.setTargetPositionGlobalFrame(position);
  simple_tracker.getTrackingVector(tracking_vector);
  ASSERT_EQ(tracking_vector, position);
}

TEST(SimpleTrackerTests, TrackingVectorActiveCameraTransform) {
  QuadSimulator drone_hardware;
  UAVVisionSystemConfig config;
  for (int i = 0; i < 6; ++i) {
    config.add_camera_transform(0.0);
  }
  config.set_camera_transform(0, 1.0);
  config.set_camera_transform(1, 2.0);
  config.set_camera_transform(2, 3.0);
  SimpleTracker simple_tracker(drone_hardware, config);
  Position position;
  Position tracking_vector;
  // Test setting tracking vector
  position = Position(1, 1, 1);
  simple_tracker.setTargetPositionGlobalFrame(position);
  simple_tracker.getTrackingVector(tracking_vector);
  ASSERT_EQ(tracking_vector, position - Position(1.0, 2.0, 3.0));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
