#include <gtest/gtest.h>

#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/trackers/simple_tracker.h"

#include <quad_simulator_parser/quad_simulator.h>

#include <glog/logging.h>

using namespace quad_simulator;
using namespace test_utils;

TEST(SimpleTrackerTests, Constructor) {
  QuadSimulator drone_hardware;
  tf::Transform camera_transform = tf::Transform::getIdentity();
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
}

TEST(SimpleTrackerTests, TrackingVector) {
  QuadSimulator drone_hardware;
  tf::Transform camera_transform = tf::Transform::getIdentity();
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
  tf::Transform pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform get_pose;
  // Test tracking is valid
  ASSERT_TRUE(simple_tracker.trackingIsValid());
  // Test setting/getting tracking vector
  simple_tracker.setTargetPoseGlobalFrame(pose);
  simple_tracker.getTrackingVector(get_pose);
  ASSERT_TF_NEAR(pose, get_pose);
}

TEST(SimpleTrackerTests, TrackingPosition) {
  QuadSimulator drone_hardware;
  tf::Transform camera_transform = tf::Transform::getIdentity();
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
  Position pos(1, 1, 1);
  tf::Transform tracking_vector;
  // Test tracking is valid
  ASSERT_TRUE(simple_tracker.trackingIsValid());
  // Test setting tracking vector
  simple_tracker.setTargetPositionGlobalFrame(pos);
  simple_tracker.getTrackingVector(tracking_vector);
  ASSERT_TF_NEAR(tracking_vector,
                 tf::Transform(tf::Quaternion(0, 0, 0, 1),
                               tf::Vector3(pos.x, pos.y, pos.z)));
}

TEST(SimpleTrackerTests, TrackingVectorActiveCameraTransform) {
  QuadSimulator drone_hardware;
  tf::Transform camera_transform = tf::Transform::getIdentity();
  camera_transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));
  SimpleTracker simple_tracker(drone_hardware, camera_transform);
  tf::Transform tracking_vector;
  // Test setting tracking vector
  tf::Transform pose =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  simple_tracker.setTargetPoseGlobalFrame(pose);
  simple_tracker.getTrackingVector(tracking_vector);
  ASSERT_TF_NEAR(tracking_vector, camera_transform.inverse() * pose);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
