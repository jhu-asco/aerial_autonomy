#include "aerial_autonomy/trackers/roi_to_position_converter.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

void RoiToPositionConverter::computeTrackingVectorWithDepth(
    std::vector<Eigen::Vector3d> &roi_position_depths,
    int number_of_depths_to_sort, const sensor_msgs::CameraInfo &camera_info,
    tf::Transform &pos) {
  const double &cx = camera_info.K[2];
  const double &cy = camera_info.K[5];
  const double &fx = camera_info.K[0];
  const double &fy = camera_info.K[4];
  Eigen::Vector3d sum(0, 0, 0);
  for (int i = 0; i < number_of_depths_to_sort; ++i) {
    sum += roi_position_depths[i];
  }
  sum *= (1.0f / number_of_depths_to_sort);
  double object_distance = sum(2);
  Eigen::Vector2d object_position_cam(0, 0);
  object_position_cam(0) = sum(0);
  object_position_cam(1) = sum(1);
  pos.setRotation(tf::Quaternion(0, 0, 0, 1));
  pos.setOrigin(tf::Vector3(
      object_distance * (object_position_cam(0) - cx) / fx,
      object_distance * (object_position_cam(1) - cy) / fy, object_distance));
}
