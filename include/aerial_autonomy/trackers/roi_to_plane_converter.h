#pragma once

#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/trackers/roi_base_tracker.h"

/**
 * @brief Converts a ROI in image to plane in camera frame
 */
class RoiToPlaneConverter : public RoiBaseTracker {
public:
  /**
   * @brief Constructor
   *
   */
  RoiToPlaneConverter(std::string name_space = "~tracker")
      : RoiBaseTracker(name_space) {}

  /**
   * @brief Get the 3D position of the plane in ROI when Depth is not empty
   * @param roi_position_depths 3D pixel points in the ROI
   * @param number_of_depths_to_sort Number of points after partial sort
   * @param camera_info Camera calibration parameters
   * @param pos Returned position
   */
  void computeTrackingVectorWithDepth(
      std::vector<Eigen::Vector3d> &roi_position_depths,
      int number_of_depths_to_sort, const sensor_msgs::CameraInfo &camera_info,
      tf::Transform &pos);

  /**
   * @brief Get the 3D point cloud of the ROI (in the frame of the
   * camera)
   * @param roi_position_depths 3D pixel points in the ROI
   * @param number_of_depths_to_sort Number of points after partial sort
   * @param cam_info Camera calibration parameters
   * @param roi_point_cloud Returned point cloud
   */
  void computePointCloud(std::vector<Eigen::Vector3d> &roi_position_depths,
                         int number_of_depths_to_sort,
                         const sensor_msgs::CameraInfo &camera_info,
                         Eigen::MatrixXd &roi_point_cloud);

  /**
   * @brief Get the pose of the plane from point cloud via Singular Value
   * Decomposition
   * @param roi_point_cloud 3byN point cloud
   * @param pos Returned pose of the plane
   */
  void computePlaneFit(Eigen::MatrixXd &roi_point_cloud, tf::Transform &pos);
};
