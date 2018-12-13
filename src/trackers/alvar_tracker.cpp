#include "aerial_autonomy/trackers/alvar_tracker.h"

#include <glog/logging.h>

bool AlvarTracker::trackingIsValid() {
  bool valid = (ros::Time::now() - last_valid_time_).toSec() < timeout_.count();
  if (!valid) {
    VLOG_EVERY_N(2, 100) << "Alvar has not been updated for "
                         << timeout_.count() << " seconds";
  }
  return valid;
}

void AlvarTracker::markerCallback(
    const ar_track_alvar_msgs::AlvarMarkers &marker_msg) {
  if (marker_msg.markers.size() == 0)
    return;
  last_valid_time_ = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  std::unordered_map<uint32_t, tf::Transform> object_poses;
  for (unsigned int i = 0; i < marker_msg.markers.size(); i++) {
    auto marker_pose = marker_msg.markers[i].pose.pose;
    tf::Transform transform(
        tf::Quaternion(marker_pose.orientation.x, marker_pose.orientation.y,
                       marker_pose.orientation.z, marker_pose.orientation.w),
        tf::Vector3(marker_pose.position.x, marker_pose.position.y,
                    marker_pose.position.z));
    object_poses[marker_msg.markers[i].id] = transform;
  }
  updateTrackingPoses(object_poses);
}

bool AlvarTracker::isConnected() { return alvar_sub_.getNumPublishers() > 0; }

std::chrono::time_point<std::chrono::high_resolution_clock>
AlvarTracker::getTrackingTime() {
  return last_tracking_time_;
}
