#include "aerial_autonomy/common/roi_to_position_converter.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

void RoiToPositionConverter::roiCallback(
    const sensor_msgs::RegionOfInterest &roi_msg) {
  last_roi_update_time_ = ros::Time::now();
  roi_rect_ = roi_msg;
}

void RoiToPositionConverter::imageCallback(
    const sensor_msgs::ImageConstPtr &img_msg) {}

void RoiToPositionConverter::cameraInfoCallback(
    const sensor_msgs::CameraInfo &cam_info_msg) {
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

  // \todo (Matt) make timeout configurable
  if ((ros::Time::now() - last_roi_update_time_).toSec() > 0.5) {
    VLOG(1) << "Last ROI update more than 0.5 seconds ago";
    object_distance_ = max_object_distance_;
    return;
  }

  cv_bridge::CvImagePtr cv_depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  std::vector<std::pair<double, Eigen::Vector2d>> roi_depths;
  for (unsigned int x = roi_rect_.x_offset;
       x < roi_rect_.x_offset + roi_rect_.width; x++) {
    for (unsigned int y = roi_rect_.y_offset;
         y < roi_rect_.y_offset + roi_rect_.height; y++) {
      float px_depth = *(cv_depth->image.ptr<float>(y, x));
      if (!isnan(px_depth) && px_depth > 0) {
        if (px_depth <= max_object_distance_)
          roi_depths.push_back(std::pair<double, Eigen::Vector2d>(
              px_depth, Eigen::Vector2d(x, y)));
      }
    }
  }

  // \todo (Matt) make perc configurable
  double perc = 0.2;
  if (roi_depths.size() == 0) {
    object_distance_ = max_object_distance_;
  } else {
    // Average of smallest "perc" percent of depths
    int number_of_depths_to_sort = int(ceil(roi_depths.size() * perc));
    std::partial_sort(roi_depths.begin(),
                      roi_depths.begin() + number_of_depths_to_sort,
                      roi_depths.end(), compare); // Doing Partial Sort
    Eigen::Vector3d sum(0, 0, 0);
    for (int i = 0; i < number_of_depths_to_sort; ++i) {
      sum += Eigen::Vector3d(roi_depths[i].second(0), roi_depths[i].second(1),
                             roi_depths[i].first);
    }
    sum *= (1.0f / number_of_depths_to_sort);
    object_distance_ = sum(2);
    object_position_cam_(0) = sum(0);
    object_position_cam_(1) = sum(1);
  }
}

bool RoiToPositionConverter::getObjectPosition(Position &pos) {
  if ((ros::Time::now() - last_roi_update_time_).toSec() > 0.5) {
    // No roi no object position
    return false;
  }
  if (!camera_info_) // Need cam info
    return false;
  double &cx = camera_info_->K[2];
  double &cy = camera_info_->K[5];
  double &fx = camera_info_->K[0];
  double &fy = camera_info_->K[4];
  if (fx == 0 || fy == 0) {
    LOG(WARNING) << "Invalid camera info";
    return false;
  }
  pos.x = object_distance_ * (object_position_cam_(0) - cx) / fx;
  pos.y = object_distance_ * (object_position_cam_(1) - cy) / fy;
  pos.z = object_distance_;
  return true;
}

bool RoiToPositionConverter::compare(std::pair<double, Eigen::Vector2d> a,
                                     std::pair<double, Eigen::Vector2d> b) {
  return (a.first < b.first);
}
