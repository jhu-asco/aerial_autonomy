#include "aerial_autonomy/trackers/roi_to_plane_converter.h"

#include <sensor_msgs/image_encodings.h>

#include <glog/logging.h>

bool RoiToPlaneConverter::isConnected() {
  return roi_subscriber_.getNumPublishers() > 0 &&
         depth_subscriber_.getNumPublishers() > 0;
}

std::chrono::time_point<std::chrono::high_resolution_clock>
RoiToPlaneConverter::getTrackingTime() {
  return last_tracking_time_;
}

void RoiToPlaneConverter::roiCallback(
    const sensor_msgs::RegionOfInterest &roi_msg) {
  last_roi_update_time_ = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  roi_rect_ = roi_msg;
}

void RoiToPlaneConverter::imageCallback(
    const sensor_msgs::ImageConstPtr &img_msg) {}

void RoiToPlaneConverter::cameraInfoCallback(
    const sensor_msgs::CameraInfo &cam_info_msg) {
  camera_info_ = cam_info_msg;
  camera_info_subscriber_.shutdown();
}

void RoiToPlaneConverter::depthCallback(
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

bool RoiToPlaneConverter::trackingIsValid() {
  return roiIsValid() && cameraInfoIsValid();
}

bool RoiToPlaneConverter::cameraInfoIsValid() {
  bool valid = camera_info_.get().K[0] != 0 && camera_info_.get().K[4] != 0;
  if (!valid)
    LOG(WARNING) << "Invalid camera info";
  return valid;
}

bool RoiToPlaneConverter::roiIsValid() {
  bool valid = (ros::Time::now() - last_roi_update_time_).toSec() < 0.5;
  if (!valid)
    VLOG(2) << "ROI has not been updated for 0.5 seconds";
  return valid;
}

bool RoiToPlaneConverter::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pos) {
  CHECK(pos.empty()) << "Tracking vector map is not empty";
  if (!trackingIsValid()) {
    return false;
  }
  pos[0] = object_pose_;
  return true;
}

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

    Eigen::MatrixXd roi_point_cloud(3, number_of_depths_to_sort);

    // Point cloud
    computePointCloud(roi_position_depths, number_of_depths_to_sort, cx, cy, fx,
                      fy, roi_point_cloud);

    // Compute Plane
    computePlaneFit(roi_point_cloud, pos);

    //
    // Eigen::Vector3d sum(0, 0, 0);
    // sum *= (1.0f / number_of_depths_to_sort);
    // object_distance = sum(2);
    // object_position_cam(0) = sum(0);
    // object_position_cam(1) = sum(1);
    // Eigen::Vector3d centroid(object_distance * (object_position_cam(0) - cx)
    // / fx, object_distance * (object_position_cam(1) - cy) / fy,
    // object_distance); roi_point_cloud.colwise() -= centroid;

    // tf::Vector3 centroid_tf;
    // tf::vectorEigenToTF(centroid, centroid_tf);
    // pos.setOrigin(tf::Vector3(centroid(0),centroid(1),centroid(2)));
    // pos.setRotation(tf::Quaternion(0, 0, 0, 1));
  }
}

void RoiToPlaneConverter::computePointCloud(
    std::vector<Eigen::Vector3d> &roi_position_depths,
    int number_of_depths_to_sort, const double &cx, const double &cy,
    const double &fx, const double &fy, Eigen::MatrixXd &roi_point_cloud) {
  for (int i = 0; i < number_of_depths_to_sort; ++i) {
    sum += roi_position_depths[i];
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

  // Singular Value Decomposition
  int setting = Eigen::ComputeFullU | Eigen::ComputeThinV;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd = roi_point_cloud.jacobiSvd(setting);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Vector3d norms = U.colwise().norm();
  U.col(0) /= norms(0);
  U.col(1) /= norms(1);
  U.col(2) /= norms(2);
  // Homogeneous transformation matrix
  Eigen::MatrixXd E;
  E.setIdentity(4, 4);
  E.block(0, 0, 3, 3) = U;
  E.block(0, 3, 3, 1) = centroid;
  conversions::transformMatrix4dToTf(E, pos);
}

bool RoiToPlaneConverter::compare(Eigen::Vector3d a, Eigen::Vector3d b) {
  return a(2) < b(2);
}
