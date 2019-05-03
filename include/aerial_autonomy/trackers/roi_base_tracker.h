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
 * @brief Interface for classes that use ROI based tracker
 */
class RoiBaseTracker : public BaseTracker {
public:
  /**
   * @brief Constructor
   *
   */
  RoiBaseTracker(std::string name_space = "~tracker")
      : BaseTracker(std::move(
            std::unique_ptr<TrackingStrategy>(new SimpleTrackingStrategy()))),
        nh_(name_space), it_(nh_),
        roi_subscriber_(
            nh_.subscribe("roi", 10, &RoiBaseTracker::roiCallback, this)),
        camera_info_subscriber_(nh_.subscribe(
            "camera_info", 1, &RoiBaseTracker::cameraInfoCallback, this)),
        depth_subscriber_(
            nh_.subscribe("depth", 1, &RoiBaseTracker::depthCallback, this)),
        image_subscriber_(
            it_.subscribe("image", 1, &RoiBaseTracker::imageCallback, this)) {
    std::cout << "ROI Base Tracker Constructor";//TAGGED
}
  /**
   * @brief Get the stored tracking vector
   * @param pos Returned tracking vectors
   * @return True if successful, false otherwise
   */
  bool getTrackingVectors(std::unordered_map<uint32_t, tf::Transform> &pos);
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
   * @brief Get the 3D position of the ROI when Depth is not empty
   * @param roi_position_depths 3D pixel points in the ROI
   * @param number_of_depths_to_sort Number of points after partial sort
   * @param camera_info Camera calibration parameters
   * @param pos Returned position
   */
  virtual void computeTrackingVectorWithDepth(
      std::vector<Eigen::Vector3d> &roi_position_depths,
      int number_of_depths_to_sort, const sensor_msgs::CameraInfo &camera_info,
      tf::Transform &pos) = 0;
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

  /**
   * @brief Get the time stamp of the current tracking vectors
   */
  virtual std::chrono::time_point<std::chrono::high_resolution_clock>
  getTrackingTime();

protected:
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
   * @brief Check whether the system has a valid Tracking vector (Pose)
   * @return True if the Pose is valid, false otherwise
   */
  bool poseIsValid();
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
   * @brief ROS node handle for communication
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
  Atomic<sensor_msgs::CameraInfo> camera_info_;
  /**
   * @brief Current roi
   */
  Atomic<sensor_msgs::RegionOfInterest> roi_rect_;
  /**
   * @brief Transform of object in camera frame (meters)
   */
  Atomic<tf::Transform> object_pose_;
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
  /**
   * @brief Last time we received a non-empty Alvar message
   */
  Atomic<std::chrono::time_point<std::chrono::high_resolution_clock>>
      last_tracking_time_;
  /**
   * @brief last time Pose was updated
   */
  Atomic<ros::Time> last_pose_update_time_;
};
