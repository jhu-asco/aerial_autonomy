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
};
