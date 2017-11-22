#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "aerial_autonomy/trackers/guidance_obstacle_tracker.h"

class GuidanceObstacleTrackerTests : public ::testing::Test {
public:
  GuidanceObstacleTrackerTests()
      : nh_(), obstacle_pub_(nh_.advertise<sensor_msgs::LaserScan>(
                   "/guidance/obstacle_distance", 1)) {}
  void publishObstacles(const std::vector<float> &ranges) {
    sensor_msgs::LaserScan obstacle_msg;
    obstacle_msg.ranges = ranges;
    obstacle_pub_.publish(obstacle_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ros::spinOnce();
  }

  void testGetTrackingVector(GuidanceObstacleTracker &tracker,
                             const std::vector<float> &ranges) {
    publishObstacles(ranges);

    std::unordered_map<uint32_t, tf::Transform> tracking_tfs;
    ASSERT_TRUE(tracker.getTrackingVectors(tracking_tfs));
    for (unsigned int i = 0; i < tracking_tfs.size(); i++) {
      ASSERT_EQ(tracking_tfs.at(i).getOrigin().z(), ranges.at(i));
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher obstacle_pub_;
};

TEST_F(GuidanceObstacleTrackerTests, TrackingValidTimeout) {
  GuidanceObstacleTracker tracker("");
  while (!tracker.isConnected()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_FALSE(tracker.trackingIsValid());

  publishObstacles({0, 0, 0, 0});
  ASSERT_TRUE(tracker.trackingIsValid());
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  ASSERT_FALSE(tracker.trackingIsValid());
}

TEST_F(GuidanceObstacleTrackerTests, GetTrackingVectors) {
  GuidanceObstacleTracker tracker("");
  while (!tracker.isConnected()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  testGetTrackingVector(tracker, {0, 2, 4, 1});
  testGetTrackingVector(tracker, {1, 5, 6, 2});

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  std::unordered_map<uint32_t, tf::Transform> tracking_tfs;
  ASSERT_FALSE(tracker.getTrackingVectors(tracking_tfs));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "guidance_obstacle_tracker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
