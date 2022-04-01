#include "aerial_autonomy/trackers/object_tracker.h"

#include <glog/logging.h>

bool ObjectTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  pose = object_poses_;
  return true;
}

bool ObjectTracker::trackingIsValid() {
  bool valid = (ros::Time::now() - last_valid_time_).toSec() < timeout_.count();
  if (!valid) {
    VLOG_EVERY_N(2, 100) << "Object detection has not been updated for "
                         << timeout_.count() << " seconds";
  }
  return valid;
}

void ObjectTracker::detectionCallback(
    const vision_msgs::Detection3DArray &detect_msg) {
  if (detect_msg.detections.size() == 0)
    return;
  last_valid_time_ = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  std::unordered_map<uint32_t, tf::Transform> object_poses;
  for (unsigned int i = 0; i < detect_msg.detections.size(); i++) {
    auto object_pose = detect_msg.detections[i].results[0].pose.pose;
    tf::Transform transform(
        tf::Quaternion(object_pose.orientation.x, object_pose.orientation.y,
                       object_pose.orientation.z, object_pose.orientation.w),
        tf::Vector3(object_pose.position.x, object_pose.position.y,
                    object_pose.position.z));
    object_poses[detect_msg.detections[i].results[0].id] = transform;
  }
  object_poses_ = object_poses;
}

bool ObjectTracker::isConnected() { return detection_sub_.getNumPublishers() > 0; }

std::chrono::time_point<std::chrono::high_resolution_clock>
ObjectTracker::getTrackingTime() {
  return last_tracking_time_;
}
