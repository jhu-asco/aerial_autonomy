#include "aerial_autonomy/trackers/KLT_tracker.h"
#include <geometry_msgs/PoseStamped.h>

#include <glog/logging.h>

bool KLTTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  pose = object_poses_;
  return true;
}

bool KLTTracker::trackingIsValid() {
  bool valid = (ros::Time::now() - last_valid_time_).toSec() < timeout_.count();
  if (!valid) {
    VLOG_EVERY_N(1, 20) << "KLT has not been updated for " << timeout_.count()
                        << " seconds";
  }
  return valid;
}

void KLTTracker::markerCallback(
    const geometry_msgs::PoseStamped &marker_msg) {
  last_valid_time_ = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  std::unordered_map<uint32_t, tf::Transform> object_poses;
  auto marker_pose = marker_msg.pose;
  tf::Transform transform(
      tf::Quaternion(marker_pose.orientation.x, marker_pose.orientation.y,
                     marker_pose.orientation.z, marker_pose.orientation.w),
      tf::Vector3(marker_pose.position.x, marker_pose.position.y,
                  marker_pose.position.z));
  object_poses[0] = transform;
  object_poses_ = object_poses;
}

bool KLTTracker::isConnected() { return klt_sub_.getNumPublishers() > 0; }

std::chrono::time_point<std::chrono::high_resolution_clock> KLTTracker::getTrackingTime() {
  return last_tracking_time_;
}
