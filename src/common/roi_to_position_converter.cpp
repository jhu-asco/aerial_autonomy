#include "aerial_autonomy/common/roi_to_position_converter.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

void RoiToPositionConverter::roiCallback(
    const sensor_msgs::RegionOfInterest &roi_msg) {
  {
    boost::mutex::scoped_lock(roi_update_mutex_);
    last_roi_update_time_ = ros::Time::now();
  }
  {
    boost::mutex::scoped_lock(roi_mutex_);
    roi_rect_ = roi_msg;
  }
}

void RoiToPositionConverter::imageCallback(
    const sensor_msgs::ImageConstPtr &img_msg) {}

void RoiToPositionConverter::cameraInfoCallback(
    const sensor_msgs::CameraInfo &cam_info_msg) {
  boost::mutex::scoped_lock(camera_info_mutex_);
  camera_info_.reset(new sensor_msgs::CameraInfo());
  *camera_info_ = cam_info_msg;
  camera_info_subscriber_.shutdown();
}

void RoiToPositionConverter::depthCallback(
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
  {
    boost::mutex::scoped_lock(roi_mutex_);
    roi_rect = roi_rect_;
  }
  sensor_msgs::CameraInfo camera_info;
  {
    boost::mutex::scoped_lock(camera_info_mutex_);
    camera_info = *camera_info_;
  }
  Position object_position;
  computeObjectPosition(roi_rect, depth->image, camera_info,
                        max_object_distance_, 0.2, object_position);
  {
    boost::mutex::scoped_lock(position_mutex_);
    object_position_ = object_position;
  }
  /// \todo store a flag indicating a position has been computed and return
  /// false in positionIsValid if it has not
}

bool RoiToPositionConverter::positionIsValid() {
  return roiIsValid() && cameraInfoIsValid();
}

bool RoiToPositionConverter::cameraInfoIsValid() {
  boost::mutex::scoped_lock(camera_info_mutex_);
  bool valid =
      camera_info_ && camera_info_->K[0] != 0 && camera_info_->K[4] != 0;
  if (!valid)
    LOG(WARNING) << "Invalid camera info";
  return valid;
}

bool RoiToPositionConverter::roiIsValid() {
  boost::mutex::scoped_lock(roi_update_mutex_);
  std::cout << (ros::Time::now() - last_roi_update_time_).toSec() << std::endl;
  bool valid = (ros::Time::now() - last_roi_update_time_).toSec() < 0.5;
  if (!valid)
    LOG(WARNING) << "ROI has not been updated for 0.5 seconds";
  return valid;
}

bool RoiToPositionConverter::getObjectPosition(Position &pos) {
  if (!roiIsValid()) {
    return false;
  }
  if (!cameraInfoIsValid()) { // Need cam info
    return false;
  }

  {
    boost::mutex::scoped_lock(position_mutex_);
    pos = object_position_;
  }
  return true;
}

void RoiToPositionConverter::computeObjectPosition(
    const sensor_msgs::RegionOfInterest &roi_rect, const cv::Mat &depth,
    const sensor_msgs::CameraInfo &camera_info, double max_distance,
    double front_percent, Position &pos) {
  std::vector<Eigen::Vector3d> roi_position_depths;
  for (unsigned int x = roi_rect.x_offset;
       x < roi_rect.x_offset + roi_rect.width; x++) {
    for (unsigned int y = roi_rect.y_offset;
         y < roi_rect.y_offset + roi_rect.height; y++) {
      float px_depth = *(depth.ptr<float>(y, x));
      if (!isnan(px_depth) && px_depth > 0) {
        if (px_depth <= max_distance)
          roi_position_depths.push_back(Eigen::Vector3d(x, y, px_depth));
      }
    }
  }

  /// \todo (Matt) make perc configurable
  double object_distance = 0;
  Eigen::Vector2d object_position_cam(0, 0);
  if (roi_position_depths.size() == 0) {
    LOG(WARNING) << "No ROI pixel depths within configured max distance";
    object_distance = max_distance;
  } else {
    // Average of smallest "front_percent" percent of depths
    /// \todo Matt Perform foreground/background clustering with k-means
    int number_of_depths_to_sort =
        int(ceil(roi_position_depths.size() * front_percent));
    std::partial_sort(roi_position_depths.begin(),
                      roi_position_depths.begin() + number_of_depths_to_sort,
                      roi_position_depths.end(), compare); // Doing Partial Sort
    Eigen::Vector3d sum(0, 0, 0);
    for (int i = 0; i < number_of_depths_to_sort; ++i) {
      sum += roi_position_depths[i];
    }
    sum *= (1.0f / number_of_depths_to_sort);
    object_distance = sum(2);
    object_position_cam(0) = sum(0);
    object_position_cam(1) = sum(1);
  }

  const double &cx = camera_info.K[2];
  const double &cy = camera_info.K[5];
  const double &fx = camera_info.K[0];
  const double &fy = camera_info.K[4];
  pos.x = object_distance * (object_position_cam(0) - cx) / fx;
  pos.y = object_distance * (object_position_cam(1) - cy) / fy;
  pos.z = object_distance;
}

bool RoiToPositionConverter::compare(Eigen::Vector3d a, Eigen::Vector3d b) {
  return a(2) < b(2);
}
