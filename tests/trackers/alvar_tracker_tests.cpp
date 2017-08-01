#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "aerial_autonomy/trackers/alvar_tracker.h"

class AlvarTrackerTests : public ::testing::Test {
public:
  AlvarTrackerTests()
      : nh_(), alvar_pub_(nh_.advertise<ar_track_alvar_msgs::AlvarMarkers>(
                   "ar_pose_marker", 1)) {}
  void publishMarkers(std::vector<std::tuple<uint32_t, Position>> markers) {
    ar_track_alvar_msgs::AlvarMarkers marker_msg;
    marker_msg.markers.resize(markers.size());
    for (unsigned int i = 0; i < markers.size(); i++) {
      marker_msg.markers[i].id = std::get<0>(markers[i]);
      marker_msg.markers[i].pose.pose.position.x = std::get<1>(markers[i]).x;
      marker_msg.markers[i].pose.pose.position.y = std::get<1>(markers[i]).y;
      marker_msg.markers[i].pose.pose.position.z = std::get<1>(markers[i]).z;
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

  std::vector<std::tuple<uint32_t, Position>> markers;
  markers.push_back(std::make_tuple(0, Position()));
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
  Position pos;

  ASSERT_FALSE(tracker.getTrackingVector(pos));

  std::vector<std::tuple<uint32_t, Position>> markers;
  markers.push_back(std::make_tuple(0, Position(0, 2, 3)));
  publishMarkers(markers);

  tracker.initialize();

  ASSERT_TRUE(tracker.getTrackingVector(pos));
  ASSERT_NEAR(pos.x, 0, 1e-6);
  ASSERT_NEAR(pos.y, 2, 1e-6);
  ASSERT_NEAR(pos.z, 3, 1e-6);
}

TEST_F(AlvarTrackerTests, TrackingValidEmpty) {
  ros::NodeHandle nh;
  AlvarTracker tracker(nh);
  while (!tracker.isConnected()) {
  }
  Position pos;

  ASSERT_FALSE(tracker.getTrackingVector(pos));

  std::vector<std::tuple<uint32_t, Position>> markers;
  markers.push_back(std::make_tuple(0, Position(0, 2, 3)));
  publishMarkers(markers);

  tracker.initialize();

  ASSERT_TRUE(tracker.getTrackingVector(pos));
  ASSERT_NEAR(pos.x, 0, 1e-6);
  ASSERT_NEAR(pos.y, 2, 1e-6);
  ASSERT_NEAR(pos.z, 3, 1e-6);

  markers.clear();
  publishMarkers(markers);
  ASSERT_FALSE(tracker.trackingIsValid());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alvar_tracker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
