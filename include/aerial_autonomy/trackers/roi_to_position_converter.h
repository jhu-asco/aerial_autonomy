#pragma once

#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/simple_tracking_strategy.h"
#include "aerial_autonomy/types/position.h"

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <Eigen/Dense>

#include <boost/thread/mutex.hpp>

/**
* @brief Converts a ROI in image to vector in camera frame
*/
class RoiToPositionConverter : public BaseTracker {
public:
  /**
  * @brief Constructor
  *
  * @param nh Nodehandle for subscribers, publishers
  */
  RoiToPositionConverter(ros::NodeHandle &nh)
      : BaseTracker(new SimpleTrackingStrategy()), it_(nh),
        roi_subscriber_(nh.subscribe(
            "roi", 10, &RoiToPositionConverter::roiCallback, this)),
        camera_info_subscriber_(
            nh.subscribe("camera_info", 1,
                         &RoiToPositionConverter::cameraInfoCallback, this)),
        depth_subscriber_(nh.subscribe(
            "depth", 1, &RoiToPositionConverter::depthCallback, this)),
        image_subscriber_(it_.subscribe(
            "image", 1, &RoiToPositionConverter::imageCallback, this)) {}

  /**
   * @brief Get the stored tracking vector
   * @param pos Returned tracking vectors
   * @return True if successful, false otherwise
   */
  bool getTrackingVectors(std::unordered_map<uint32_t, Position> &pos);
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
  static void computeTrackingVector(const sensor_msgs::RegionOfInterest &roi,
                                    const cv::Mat &depth,
                                    const sensor_msgs::CameraInfo &cam_info,
                                    double max_distance,
                                    double foreground_percent, Position &pos);
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  bool trackingIsValid();

  /**
  * @brief Check whether topics are connected
  * @return True if connected and false otherwise
  */
  bool isConnected();

private:
  /**
  * @brief Check whether the system has valid camera info
  * @return True if the camera info is valid, false otherwise
  */
  bool cameraInfoIsValid();
  /**
  * @brief Check whether the system has a valid ROI
  * @return True if the ROI is valid, false otherwise
  */
  bool roiIsValid();
  /**
   * @brief ROI subscriber callback
   * @param roi_msg ROI message
   */
  void roiCallback(const sensor_msgs::RegionOfInterest &roi_msg);

  /**
   * @brief Image subscriber callback
   * @param img_msg Image message
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &img_msg);

  /**
   * @brief CameraInfo subscriber callback
   * @param cam_info_msg Image message
   */
  void cameraInfoCallback(const sensor_msgs::CameraInfo &cam_info_msg);

  /**
   * @brief Depth subscriber callback
   * @param depth_msg Depth message
   */
  void depthCallback(const sensor_msgs::ImageConstPtr &depth_msg);

  /**
  * @brief Comparator for sorting points by depth
  * @param a First point
  * @param b Second point
  * @return True if a depth is less than b depth
  */
  static bool compare(Eigen::Vector3d a, Eigen::Vector3d b);

  /**
  * @brief Image transport
  */
  image_transport::ImageTransport it_;
  /**
  * @brief ROS subscriber for receiving region of interest
  */
  ros::Subscriber roi_subscriber_;
  /**
  * @brief ROS subscriber for receiving camera info
  */
  ros::Subscriber camera_info_subscriber_;
  /**
  * @brief ROS subscriber for receiving depth
  */
  ros::Subscriber depth_subscriber_;
  /**
  * @brief ROS subscriber for receiving image
  */
  image_transport::Subscriber image_subscriber_;
  /**
  * @brief Camera info for conversion to 3D
  */
  Atomic<sensor_msgs::CameraInfo> camera_info_;
  /**
  * @brief Current roi
  */
  Atomic<sensor_msgs::RegionOfInterest> roi_rect_;
  /**
  * @brief Position of object in camera frame (meters)
  */
  Atomic<Position> object_position_;
  /**
  * @brief Max distance of object from camera (meters)
  * \todo Make this a configurable param
  */
  double max_object_distance_ = 3.0;
  /**
  * @brief Percentage of pixel depths to include as foreground
  * \todo Make this a configurable param
  */
  double foreground_percent_ = 0.25;

  /**
  * @brief last time ROI was updated
  */
  Atomic<ros::Time> last_roi_update_time_;
};
