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
  auto begin_time = ros::Time::now();
  double depth_multiplier = 1.0;
  if (depth_msg->encoding == std::string("16UC1")) {
    depth_multiplier = .001;
  } else if (depth_msg->encoding != std::string("32FC1")) {
    LOG(ERROR) << "Depth encoding " << depth_msg->encoding << " not expected";
    return;
  }
  last_pose_update_time_ = ros::Time::now();
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
  computeTrackingVector(roi_rect, depth_multiplier * depth->image, camera_info,
                        max_object_distance_, foreground_percent_, object_pose);
  publishPose(object_pose);
  object_pose_ = object_pose;
  double callback_time = (ros::Time::now() - begin_time).toSec();
  if (callback_time > 0.5) {
    LOG(WARNING) << "Depth Callback took  " << callback_time << " seconds";
  }
}

// TODO (Matt): Move to common file
void RoiBaseTracker::publishPose(const tf::Transform &pose) {
  auto origin = pose.getOrigin();
  auto stamp = ros::Time::now();

  visualization_msgs::MarkerArray markers;
  for (unsigned int i = 0; i < 3; i++) {
    visualization_msgs::Marker marker;
    auto axis = pose.getBasis().getColumn(i);

    marker.header.frame_id = "camera";
    marker.header.stamp = stamp;
    marker.ns = "tracker_pose";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = i == 0 ? 1.0 : 0;
    marker.color.g = i == 1 ? 1.0 : 0;
    marker.color.b = i == 2 ? 1.0 : 0;

    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();
    auto end_tf = origin + axis;
    geometry_msgs::Point end;
    end.x = end_tf.x();
    end.y = end_tf.y();
    end.z = end_tf.z();
    marker.points.push_back(start);
    marker.points.push_back(end);

    markers.markers.push_back(marker);
  }
  pose_publisher_.publish(markers);
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
  double update_time = (ros::Time::now() - last_roi_update_time_).toSec();
  bool valid = update_time < 0.5;
  if (!valid) {
    VLOG_EVERY_N(2, 20) << "ROI has not been updated for " << update_time
                        << " seconds";
  }
  return valid;
}

bool RoiBaseTracker::poseIsValid() {
  bool valid = (ros::Time::now() - last_pose_update_time_).toSec() < 0.5;
  if (!valid) {
    VLOG_EVERY_N(2, 20) << "Pose has not been updated for 0.5 seconds";
  }
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
      if (y >= 0 && int(y) < depth.rows && x >= 0 && int(x) < depth.cols) {
        float px_depth = *(depth.ptr<float>(y, x));
        if (!std::isnan(px_depth) && px_depth > 0) {
          if (px_depth <= max_distance)
            roi_position_depths.push_back(Eigen::Vector3d(x, y, px_depth));
        }
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
