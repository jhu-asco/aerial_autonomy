#include <gtest/gtest.h>

#include "aerial_autonomy/trackers/simple_tracker.h"

#include <quad_simulator_parser/quad_simulator.h>

#include <glog/logging.h>

using namespace quad_simulator;

TEST(SimpleTrackerTests, Constructor) {
  QuadSimulator drone_hardware;
  tf::Transform camera_transform = tf::Transform::getIdentity();
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
}

TEST(SimpleTrackerTests, TrackingVector) {
  QuadSimulator drone_hardware;
  tf::Transform camera_transform = tf::Transform::getIdentity();
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
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
  tf::Transform camera_transform = tf::Transform::getIdentity();
  camera_transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
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
