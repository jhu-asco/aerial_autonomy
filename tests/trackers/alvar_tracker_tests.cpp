#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/trackers/alvar_tracker.h"

using namespace test_utils;

class AlvarTrackerTests : public ::testing::Test {
public:
  AlvarTrackerTests()
      : nh_(), alvar_pub_(nh_.advertise<ar_track_alvar_msgs::AlvarMarkers>(
                   "ar_pose_marker", 1)) {}
  void
  publishMarkers(std::vector<std::tuple<uint32_t, tf::Transform>> markers) {
    ar_track_alvar_msgs::AlvarMarkers marker_msg;
    marker_msg.markers.resize(markers.size());
    for (unsigned int i = 0; i < markers.size(); i++) {
      marker_msg.markers[i].id = std::get<0>(markers[i]);
      auto position = std::get<1>(markers[i]).getOrigin();
      marker_msg.markers[i].pose.pose.position.x = position.x();
      marker_msg.markers[i].pose.pose.position.y = position.y();
      marker_msg.markers[i].pose.pose.position.z = position.z();
      auto rotation = std::get<1>(markers[i]).getRotation();
      marker_msg.markers[i].pose.pose.orientation.x = rotation.x();
      marker_msg.markers[i].pose.pose.orientation.y = rotation.y();
      marker_msg.markers[i].pose.pose.orientation.z = rotation.z();
      marker_msg.markers[i].pose.pose.orientation.w = rotation.w();
    }
    alvar_pub_.publish(marker_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ros::spinOnce();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher alvar_pub_;
};

TEST_F(AlvarTrackerTests, TrackingValidTimeout) {
  ros::NodeHandle nh;
  AlvarTracker tracker(nh);
  while (!tracker.isConnected()) {
  }

  ASSERT_FALSE(tracker.trackingIsValid());

  std::vector<std::tuple<uint32_t, tf::Transform>> markers;
  markers.push_back(std::make_tuple(0, tf::Transform()));
  publishMarkers(markers);
  ASSERT_TRUE(tracker.trackingIsValid());

  /// \todo Matt This should depend on the configured ROI timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  ASSERT_FALSE(tracker.trackingIsValid());
}

TEST_F(AlvarTrackerTests, GetTrackingVector) {
  ros::NodeHandle nh;
  AlvarTracker tracker(nh);
  while (!tracker.isConnected()) {
  }
  tf::Transform pose;

  ASSERT_FALSE(tracker.getTrackingVector(pose));

  std::vector<std::tuple<uint32_t, tf::Transform>> markers;
  markers.push_back(std::make_tuple(
      0, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 2, 3))));
  publishMarkers(markers);

  tracker.initialize();

  ASSERT_TRUE(tracker.getTrackingVector(pose));
  ASSERT_TF_NEAR(
      pose, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 2, 3)));
}

TEST_F(AlvarTrackerTests, TrackingValidEmpty) {
  ros::NodeHandle nh;
  AlvarTracker tracker(nh);
  while (!tracker.isConnected()) {
  }
  tf::Transform pose;

  ASSERT_FALSE(tracker.getTrackingVector(pose));

  std::vector<std::tuple<uint32_t, tf::Transform>> markers;
  markers.push_back(std::make_tuple(
      0, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 2, 3))));
  publishMarkers(markers);

  tracker.initialize();

  ASSERT_TRUE(tracker.getTrackingVector(pose));
  ASSERT_TF_NEAR(
      pose, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 2, 3)));

  markers.clear();
  publishMarkers(markers);
  // Should still be valid until the timeout hits
  ASSERT_TRUE(tracker.trackingIsValid());

  /// \todo Matt This should depend on the configured ROI timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  publishMarkers(markers);
  ASSERT_FALSE(tracker.trackingIsValid());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alvar_tracker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
