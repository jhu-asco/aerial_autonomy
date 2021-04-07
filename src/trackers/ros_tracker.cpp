#include "aerial_autonomy/trackers/ros_tracker.h"

#include <glog/logging.h>

bool ROSTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  pose = object_poses_;
  return true;
}

bool ROSTracker::trackingIsValid() {
  bool valid = (ros::Time::now() - last_valid_time_).toSec() < timeout_.count();
  if (!valid) {
    VLOG_EVERY_N(2, 100) << "Tracking has not been updated for "
                         << timeout_.count() << " seconds";
  }
  return valid;
}

void ROSTracker::markerCallback(
    const geometry_msgs::PoseArray &pose_msg) {
  if (pose_msg.poses.size() == 0)
    return;
  last_valid_time_ = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  std::unordered_map<uint32_t, tf::Transform> object_poses;
  for (unsigned int i = 0; i < pose_msg.poses.size(); i++) {
    auto pose = pose_msg.poses[i];
    tf::Transform transform(
        tf::Quaternion(pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w),
        tf::Vector3(pose.position.x, pose.position.y,
                    pose.position.z));
    object_poses[i] = transform;//Change this if tracking multiple things
  }
  object_poses_ = object_poses;
}

bool ROSTracker::isConnected() { return tracking_sub_.getNumPublishers() > 0; }

std::chrono::time_point<std::chrono::high_resolution_clock>
ROSTracker::getTrackingTime() {
  return last_tracking_time_;
}
