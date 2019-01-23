#include "aerial_autonomy/trackers/roi_to_position_converter.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

void RoiToPositionConverter::computeTrackingVector(
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

  /// \todo (Matt) make perc configurable
  double object_distance = 0;
  Eigen::Vector2d object_position_cam(0, 0);
  if (roi_position_depths.size() == 0) {
    VLOG(2) << "No ROI pixel depths within configured max distance";
    object_distance = max_distance;
    object_position_cam(0) = roi_rect.x_offset + roi_rect.width / 2.;
    object_position_cam(1) = roi_rect.y_offset + roi_rect.height / 2.;
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
  pos.setRotation(tf::Quaternion(0, 0, 0, 1));
  pos.setOrigin(tf::Vector3(
      object_distance * (object_position_cam(0) - cx) / fx,
      object_distance * (object_position_cam(1) - cy) / fy, object_distance));
}
