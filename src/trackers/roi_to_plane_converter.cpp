#include "aerial_autonomy/trackers/roi_to_plane_converter.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

void RoiToPlaneConverter::computeTrackingVectorWithDepth(
    std::vector<Eigen::Vector3d> &roi_position_depths,
    int number_of_depths_to_sort, const sensor_msgs::CameraInfo &camera_info,
    tf::Transform &pos) {
  // 3byN
  Eigen::MatrixXd roi_point_cloud(3, number_of_depths_to_sort);
  // Point cloud
  computePointCloud(roi_position_depths, number_of_depths_to_sort, camera_info,
                    roi_point_cloud);
  // Compute Plane
  computePlaneFit(roi_point_cloud, pos);
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
  // Make normal vector to be x-axis of that is coming out from the plane
  Eigen::Vector3d normal_vec = U.col(2);
  if (normal_vec(2) > 0) {
    normal_vec = -normal_vec;
  }
  // Cross product with x-axis of the camera (just (1,0,0)) to get z-axis
  // (a,b,c) X (1,0,0) = (0,c,-b)
  Eigen::Vector3d vertical_vec(0, normal_vec(2), -normal_vec(1));
  // Finally, cross product two axes to get y-axis
  Eigen::Vector3d horiz_vec = vertical_vec.cross(normal_vec);
  Eigen::Matrix3d R;
  R << normal_vec.normalized(), horiz_vec.normalized(),
      vertical_vec.normalized();
  // Homogeneous transformation matrix
  Eigen::MatrixXd E = Eigen::MatrixXd::Identity(4, 4);
  E.block(0, 0, 3, 3) = R;
  E.block(0, 3, 3, 1) = centroid;
  conversions::transformMatrix4dToTf(E, pos);
}
