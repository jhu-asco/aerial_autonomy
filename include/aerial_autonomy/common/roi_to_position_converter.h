#pragma once

#include "aerial_autonomy/types/position.h"

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <Eigen/Dense>

class RoiToPositionConverter {
public:
  RoiToPositionConverter(ros::NodeHandle &nh)
      : nh_(nh), it_(nh_),
        roi_subscriber_(nh_.subscribe(
            "roi", 10, &RoiToPositionConverter::roiCallback, this)),
        camera_info_subscriber_(
            nh_.subscribe("camera_info", 1,
                          &RoiToPositionConverter::cameraInfoCallback, this)),
        depth_subscriber_(nh_.subscribe(
            "depth", 1, &RoiToPositionConverter::depthCallback, this)),
        image_subscriber_(it_.subscribe(
            "image", 1, &RoiToPositionConverter::imageCallback, this)) {}
  /**
   * @brief Get the position of the object in the ROI (in the frame of the
   * camera)
   * @param pos Returned position
   * @return Return true if successful and false otherwise
   */
  bool getObjectPosition(Position &pos);

private:
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
  static bool compare(std::pair<double, Eigen::Vector2d> a,
                      std::pair<double, Eigen::Vector2d> b);

  /**
  * @brief ROS node handle for receiving ROI
  */
  ros::NodeHandle nh_;
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
  sensor_msgs::CameraInfoPtr camera_info_;
  /**
  * @brief Current roi
  */
  sensor_msgs::RegionOfInterest roi_rect_;
  /**
  * @brief Object 2D position in camera (pixels)
  */
  Eigen::Vector2d object_position_cam_;
  /**
  * @brief Distance of object from camera (meters)
  */
  double object_distance_;
  /**
  * @brief Max distance of object from camera (meters)
  * \todo Make this a configurable param
  */
  double max_object_distance_ = 3.0;

  /**
  * @brief last time ROI was updated
  */
  ros::Time last_roi_update_time_;
};
