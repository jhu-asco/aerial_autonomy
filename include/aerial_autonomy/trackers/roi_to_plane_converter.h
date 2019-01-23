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
   * @brief Get the 3D position of the ROI (in the frame of the
   * camera)
   * @param roi Region of interest to consider
   * @param depth Pixel depths
   * @param cam_info Camera calibration parameters
   * @param max_distance Ignore pixels farther away than this
   * @param foreground_percent Average over closest foreground_percent of pixels
   * @param pos Returned position
   */
  void computeTrackingVector(const sensor_msgs::RegionOfInterest &roi,
                             const cv::Mat &depth,
                             const sensor_msgs::CameraInfo &cam_info,
                             double max_distance, double foreground_percent,
                             tf::Transform &pos);

  /**
   * @brief Get the 3D point cloud of the ROI (in the frame of the
   * camera)
   * @param roi_position_depths
   * @param number_of_depths_to_sort
   * @param cam_info Camera calibration parameters
   * @param roi_point_cloud Returned point cloud
   */
  void computePointCloud(std::vector<Eigen::Vector3d> &roi_position_depths,
                         int number_of_depths_to_sort,
                         const sensor_msgs::CameraInfo &camera_info,
                         Eigen::MatrixXd &roi_point_cloud);

  /**
   * @brief Get the 3D plane fitting via singular value decomposition method
   * @param roi_point_cloud 3byN point cloud
   * @param roi_point_cloud Returned pos
   */
  void computePlaneFit(Eigen::MatrixXd &roi_point_cloud, tf::Transform &pos);
};
