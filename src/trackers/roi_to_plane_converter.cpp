#include "aerial_autonomy/trackers/roi_to_plane_converter.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

void RoiToPlaneConverter::computeTrackingVector(
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
    // 3byN
    Eigen::MatrixXd roi_point_cloud(3, number_of_depths_to_sort);
    // Point cloud
    computePointCloud(roi_position_depths, number_of_depths_to_sort,
                      camera_info, roi_point_cloud);
    // Compute Plane
    computePlaneFit(roi_point_cloud, pos);
  }
}

void RoiToPlaneConverter::computePointCloud(
    std::vector<Eigen::Vector3d> &roi_position_depths,
    int number_of_depths_to_sort, const sensor_msgs::CameraInfo &camera_info,
    Eigen::MatrixXd &roi_point_cloud) {
  const double &cx = camera_info.K[2];
  const double &cy = camera_info.K[5];
  const double &fx = camera_info.K[0];
  const double &fy = camera_info.K[4];
  for (int i = 0; i < number_of_depths_to_sort; ++i) {
    roi_point_cloud(0, i) =
        roi_position_depths[i](2) * (roi_position_depths[i](0) - cx) / fx;
    roi_point_cloud(1, i) =
        roi_position_depths[i](2) * (roi_position_depths[i](1) - cy) / fy;
    roi_point_cloud(2, i) = roi_position_depths[i](2);
  }
}

void RoiToPlaneConverter::computePlaneFit(Eigen::MatrixXd &roi_point_cloud,
                                          tf::Transform &pos) {
  // Compute centroid
  Eigen::Vector3d centroid = roi_point_cloud.rowwise().mean();
  roi_point_cloud.colwise() -= centroid;
  // Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      roi_point_cloud, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Vector3d norms = U.colwise().norm();
  U.col(0) /= norms(0);
  U.col(1) /= norms(1);
  U.col(2) /= norms(2);
  // Homogeneous transformation matrix
  Eigen::MatrixXd E = Eigen::MatrixXd::Identity(4, 4);
  E.block(0, 0, 3, 3) = U;
  E.block(0, 3, 3, 1) = centroid;
  conversions::transformMatrix4dToTf(E, pos);
}
