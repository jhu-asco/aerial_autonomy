#include "aerial_autonomy/trackers/roi_base_tracker.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

bool RoiBaseTracker::isConnected() {
  return roi_subscriber_.getNumPublishers() > 0 &&
         depth_subscriber_.getNumPublishers() > 0;
}

std::chrono::time_point<std::chrono::high_resolution_clock>
RoiBaseTracker::getTrackingTime() {
  return last_tracking_time_;
}

void RoiBaseTracker::roiCallback(const sensor_msgs::RegionOfInterest &roi_msg) {
  last_roi_update_time_ = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  roi_rect_ = roi_msg;
}

void RoiBaseTracker::imageCallback(const sensor_msgs::ImageConstPtr &img_msg) {}

void RoiBaseTracker::cameraInfoCallback(
    const sensor_msgs::CameraInfo &cam_info_msg) {
  camera_info_ = cam_info_msg;
  camera_info_subscriber_.shutdown();
}

void RoiBaseTracker::depthCallback(
    const sensor_msgs::ImageConstPtr &depth_msg) {
  if (depth_msg->encoding != std::string("32FC1")) {
    LOG(ERROR) << "Depth encoding " << depth_msg->encoding << " not expected";
    return;
  }

  /// \todo (Matt) make timeout configurable
  if (!roiIsValid()) {
    return;
  }

  cv_bridge::CvImagePtr depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

  if (!cameraInfoIsValid()) {
    return;
  }

  // Copy variables
  sensor_msgs::RegionOfInterest roi_rect;
  roi_rect = roi_rect_;
  sensor_msgs::CameraInfo camera_info;
  camera_info = camera_info_;
  tf::Transform object_pose;
  computeTrackingVector(roi_rect, depth->image, camera_info,
                        max_object_distance_, foreground_percent_, object_pose);
  object_pose_ = object_pose;
  /// \todo store a flag indicating a position has been computed and return
  /// false in positionIsValid if it has not
}

bool RoiBaseTracker::trackingIsValid() {
  return roiIsValid() && cameraInfoIsValid();
}

bool RoiBaseTracker::cameraInfoIsValid() {
  bool valid = camera_info_.get().K[0] != 0 && camera_info_.get().K[4] != 0;
  if (!valid)
    LOG(WARNING) << "Invalid camera info";
  return valid;
}

bool RoiBaseTracker::roiIsValid() {
  bool valid = (ros::Time::now() - last_roi_update_time_).toSec() < 0.5;
  if (!valid)
    VLOG(2) << "ROI has not been updated for 0.5 seconds";
  return valid;
}

bool RoiBaseTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pos) {
  CHECK(pos.empty()) << "Tracking vector map is not empty";
  if (!trackingIsValid()) {
    return false;
  }
  pos[0] = object_pose_;
  return true;
}

bool RoiBaseTracker::compare(Eigen::Vector3d a, Eigen::Vector3d b) {
  return a(2) < b(2);
}
