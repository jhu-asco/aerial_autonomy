#pragma once

#include "aerial_autonomy/trackers/roi_base_tracker.h"

/**
 * @brief Converts a ROI in image to vector in camera frame
 */
class RoiToPositionConverter : public RoiBaseTracker {
public:
  /**
   * @brief Constructor
   *
   */
  RoiToPositionConverter(std::string name_space = "~tracker")
      : RoiBaseTracker(name_space) {}

  /**
   * @brief Get the 3D position of the ROI when Depth is not empty
   * @param roi_position_depths
   * @param number_of_depths_to_sort
   * @param camera_info Camera calibration parameters
   * @param pos Returned position
   */
  void computeTrackingVectorWithDepth(
      std::vector<Eigen::Vector3d> &roi_position_depths,
      int number_of_depths_to_sort, const sensor_msgs::CameraInfo &camera_info,
      tf::Transform &pos);
};
