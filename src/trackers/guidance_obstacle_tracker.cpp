#include "aerial_autonomy/trackers/guidance_obstacle_tracker.h"

bool GuidanceObstacleTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pos) {
  bool tracking_valid = trackingIsValid();
  if (tracking_valid) {
    pos.clear();
    std::vector<tf::Transform> obstacle_transforms = obstacle_transforms;
    for (unsigned int i = 0; i < obstacle_transforms.size(); i++) {
      pos[i] = obstacle_transforms[i];
    }
  }
  return tracking_valid;
}

bool GuidanceObstacleTracker::trackingIsValid() {
  return (ros::Time::now() - last_obstacle_time_).toSec() < msg_timeout_;
}

void GuidanceObstacleTracker::guidanceCallback(
    const sensor_msgs::LaserScan &obstacle_msg) {
  last_obstacle_time_ = ros::Time::now();
  std::vector<tf::Transform> obstacle_transforms(obstacle_msg.ranges.size());
  for (unsigned int i = 0; i < obstacle_msg.ranges.size(); i++) {
    // \todo Matt transform based on camera transforms
    obstacle_transforms[i] = tf::Transform(
        tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, obstacle_msg.ranges[i]));
  }
  obstacle_transforms_ = obstacle_transforms;
}
