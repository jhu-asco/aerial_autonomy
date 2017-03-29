#include "aerial_autonomy/common/roi_to_position_converter.h"

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

  cv_bridge::CvImagePtr depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  // \todo Matt lock this to avoid conflict with getObjectPosition
  if (!computeObjectPosition(roi_rect_, depth->image, *camera_info_,
                             max_object_distance_, 0.2, object_position_)) {
    // \todo Matt handle this case
  }
  object_distance_ = object_position_.z;
}

bool RoiToPositionConverter::getObjectPosition(Position &pos) {
  if ((ros::Time::now() - last_roi_update_time_).toSec() > 0.5) {
    // No roi no object position
    return false;
  }
  if (!camera_info_) // Need cam info
    return false;

  // \todo Matt lock this to avoid conflict with depthCallback
  pos = object_position_;
  return true;
}

bool RoiToPositionConverter::computeObjectPosition(
    const sensor_msgs::RegionOfInterest &roi_rect, const cv::Mat &depth,
    const sensor_msgs::CameraInfo &camera_info, double max_distance,
    double front_percent, Position &pos) {
  std::vector<std::pair<double, Eigen::Vector2d>> roi_depths;
  for (unsigned int x = roi_rect.x_offset;
       x < roi_rect.x_offset + roi_rect.width; x++) {
    for (unsigned int y = roi_rect.y_offset;
         y < roi_rect.y_offset + roi_rect.height; y++) {
      float px_depth = *(depth.ptr<float>(y, x));
      if (!isnan(px_depth) && px_depth > 0) {
        if (px_depth <= max_distance)
          roi_depths.push_back(std::pair<double, Eigen::Vector2d>(
              px_depth, Eigen::Vector2d(x, y)));
      }
    }
  }

  // \todo (Matt) make perc configurable
  double object_distance = 0;
  Eigen::Vector2d object_position_cam(0, 0);
  if (roi_depths.size() == 0) {
    object_distance = max_distance;
  } else {
    // Average of smallest "front_percent" percent of depths
    int number_of_depths_to_sort = int(ceil(roi_depths.size() * front_percent));
    std::partial_sort(roi_depths.begin(),
                      roi_depths.begin() + number_of_depths_to_sort,
                      roi_depths.end(), compare); // Doing Partial Sort
    Eigen::Vector3d sum(0, 0, 0);
    for (int i = 0; i < number_of_depths_to_sort; ++i) {
      sum += Eigen::Vector3d(roi_depths[i].second(0), roi_depths[i].second(1),
                             roi_depths[i].first);
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
  if (fx == 0 || fy == 0) {
    LOG(WARNING) << "Invalid camera info";
    return false;
  }
  pos.x = object_distance * (object_position_cam(0) - cx) / fx;
  pos.y = object_distance * (object_position_cam(1) - cy) / fy;
  pos.z = object_distance;
  return true;
}

bool RoiToPositionConverter::compare(std::pair<double, Eigen::Vector2d> a,
                                     std::pair<double, Eigen::Vector2d> b) {
  return (a.first < b.first);
}
