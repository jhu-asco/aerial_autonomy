#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include "aerial_autonomy/common/atomic.h"

#include <geometry_msgs/PoseArray.h>
#include <chrono>
#include <ros/ros.h>

/**
 * @brief Provides transform to a tracked object based on output from a ROS topic
 */
class ROSTracker : public BaseTracker {
public:
  /**
  * @brief Constructor.  Use the ClosestTrackingStrategy by default.
  * @param name_space Namespace of internal ROS node handle
  */
  ROSTracker(
      std::string name_space = "~tracker",
      std::chrono::duration<double> timeout = std::chrono::milliseconds(500),
      std::string topic_name = "ros_tracker")
      : BaseTracker(std::move(std::unique_ptr<TrackingStrategy>(
            new ClosestTrackingStrategy(default_num_retries_)))),
        nh_(name_space),
        tracking_sub_(nh_.subscribe(topic_name, 1,
                                 &ROSTracker::markerCallback, this)),
        timeout_(timeout) {
          ROS_INFO_STREAM("ROS Tracker Constructed.  topic name: " << topic_name);
          }
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
  * @brief Get the time stamp of the current tracking vectors
  */
  virtual std::chrono::time_point<std::chrono::high_resolution_clock>
  getTrackingTime();

  /**
  * @brief Check if subscriber is connected
  * @return True if connected, false otherwise
  */
  bool isConnected();

private:
  /**
  * @brief Marker subscriber callback
  * @param marker_msg Marker message
  */
  void markerCallback(const geometry_msgs::PoseArray &pose_msg);

  /**
  * @brief ROS node handle for communication
  */
  ros::NodeHandle nh_;
  /**
  * @brief Marker subscriber
  */
  ros::Subscriber tracking_sub_;
  /**
  * @brief Last time we received a non-empty ROS message
  */
  Atomic<ros::Time> last_valid_time_;
  /**
  * @brief Last time we received a non-empty ROS message
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
  const std::chrono::duration<double> timeout_;
  /**
  * @brief Default number of retries for tracking a locked target
  */
  const int default_num_retries_ = 25;
};
