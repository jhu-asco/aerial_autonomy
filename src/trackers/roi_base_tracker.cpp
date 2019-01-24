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
  last_pose_update_time_ = ros::Time::now();
  if (!poseIsValid()) {
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
}

bool RoiBaseTracker::trackingIsValid() {
  return roiIsValid() && cameraInfoIsValid() && poseIsValid();
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

bool RoiBaseTracker::poseIsValid() {
  bool valid = (ros::Time::now() - last_pose_update_time_).toSec() < 0.5;
  if (!valid)
    VLOG(2) << "Pose has not been updated for 0.5 seconds";
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

void RoiBaseTracker::computeTrackingVector(
    const sensor_msgs::RegionOfInterest &roi_rect, const cv::Mat &depth,
    const sensor_msgs::CameraInfo &camera_info, double max_distance,
    double front_percent, tf::Transform &pos) {
  std::vector<Eigen::Vector3d> roi_position_depths;
  for (unsigned int x = roi_rect.x_offset;
       x < roi_rect.x_offset + roi_rect.width; x++) {
    for (unsigned int y = roi_rect.y_offset;
         y < roi_rect.y_offset + roi_rect.height; y++) {
      float px_depth = *(depth.ptr<float>(y, x));
      if (!std::isnan(px_depth) && px_depth > 0) {
        if (px_depth <= max_distance)
          roi_position_depths.push_back(Eigen::Vector3d(x, y, px_depth));
      }
    }
  }
  const double &cx = camera_info.K[2];
  const double &cy = camera_info.K[5];
  const double &fx = camera_info.K[0];
  const double &fy = camera_info.K[4];

  /// \todo (Matt) make perc configurable
  double object_distance = 0;
  Eigen::Vector2d object_position_cam(0, 0);
  if (roi_position_depths.size() == 0) {
    VLOG(2) << "No ROI pixel depths within configured max distance";
    object_distance = max_distance;
    object_position_cam(0) = roi_rect.x_offset + roi_rect.width / 2.;
    object_position_cam(1) = roi_rect.y_offset + roi_rect.height / 2.;
    pos.setRotation(tf::Quaternion(0, 0, 0, 1));
    pos.setOrigin(tf::Vector3(
        object_distance * (object_position_cam(0) - cx) / fx,
        object_distance * (object_position_cam(1) - cy) / fy, object_distance));
  } else {
    // Average of smallest "front_percent" percent of depths
    /// \todo Matt Perform foreground/background clustering with k-means
    int number_of_depths_to_sort =
        int(ceil(roi_position_depths.size() * front_percent));
    std::partial_sort(roi_position_depths.begin(),
                      roi_position_depths.begin() + number_of_depths_to_sort,
                      roi_position_depths.end(), compare); // Doing Partial Sort

    computeTrackingVectorWithDepth(roi_position_depths,
                                   number_of_depths_to_sort, camera_info, pos);
  }
}
