#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include "aerial_autonomy/common/atomic.h"

#include <vision_msgs/Detection3DArray.h>
#include <chrono>
#include <ros/ros.h>

/**
 * @brief Provides transform to a tracked object based on output from object detector
 */
class ObjectTracker : public BaseTracker {
public:
  /**
  * @brief Constructor.  Use the ClosestTrackingStrategy by default.
  * @param timeout Timeout before tracking is invalid
  * @param name_space Namespace of internal ROS node handle
  */
  ObjectTracker(
      std::chrono::duration<double> timeout = std::chrono::milliseconds(500),
      std::string name_space = "~tracker")
      : BaseTracker(std::move(std::unique_ptr<TrackingStrategy>(
            new ClosestTrackingStrategy(default_num_retries_)))),
        nh_(name_space),
        detection_sub_(nh_.subscribe("object_detections", 1,
                                 &ObjectTracker::detectionCallback, this)),
        timeout_(timeout), id_factor_(100) {}

  /**
   * @brief Get the tracking vectors
   * @param pos Returned tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool
  getTrackingVectors(std::unordered_map<uint32_t, tf::Transform> &pos);

  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid();

  /**
  * @brief Check whether tracking is valid and remove invalid poses
  * @return True if the tracking is valid, false otherwise
  */
  bool trackingIsValidAndRemoveInvalidPoses(
    std::unordered_map<uint32_t, tf::Transform> &object_poses,
    std::unordered_map<uint32_t, ros::Time> &last_valid_times,
    std::vector<uint32_t> &removed_objects);

  /**
  * @brief Get the time stamp of the current tracking vectors
  */
  virtual std::chrono::time_point<std::chrono::high_resolution_clock>
  getTrackingTime();

  /**
  * @brief Check if subscriber is connected
  * @return True if connected, false otherwise
  */
  bool isConnected();

  /**
  * @brief Get the new tracking transforms
  */
  std::unordered_map<uint32_t, tf::Transform>
  getObjectPoses(const vision_msgs::Detection3DArray &detect_msg);

  std::chrono::duration<double>
  getTimeout()
  {
    return timeout_;
  }

  void
  setTimeout(std::chrono::duration<double> new_timeout)
  {
    timeout_ = new_timeout;
  }

protected:
  /**
  * @brief Detection subscriber callback
  * @param msg Detection array message
  */
  void detectionCallback(const vision_msgs::Detection3DArray &detect_msg);

  /**
  * @brief ROS node handle for communication
  */
  ros::NodeHandle nh_;
  /**
  * @brief Detection subscriber
  */
  ros::Subscriber detection_sub_;
  /**
  * @brief Last time we received a non-empty Detection message
  */
  Atomic<std::unordered_map<uint32_t, ros::Time>> last_valid_times_;
  /**
  * @brief Last time we received a non-empty Detection message
  */
  Atomic<std::chrono::time_point<std::chrono::high_resolution_clock>>
      last_tracking_time_;
  /**
  * @brief Stored tracking transforms
  */
  Atomic<std::unordered_map<uint32_t, tf::Transform>> object_poses_;
  /**
  * @brief Timeout for valid update
  */
  std::chrono::duration<double> timeout_;
  /**
  * @brief Default number of retries for tracking a locked target
  */
  const int default_num_retries_ = 25;
  /**
  * @brief Factor to use for separate objects of the same ID
  */
  int id_factor_; 

};
