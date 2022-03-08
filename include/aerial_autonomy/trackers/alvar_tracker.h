#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include "aerial_autonomy/common/atomic.h"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <chrono>
#include <ros/ros.h>

/**
 * @brief Provides transform to a tracked object based on output from alvar
 */
class AlvarTracker : public BaseTracker {
public:
  /**
  * @brief Constructor.  Use the ClosestTrackingStrategy by default.
  * @param name_space Namespace of internal ROS node handle
  */
  AlvarTracker(
      std::string name_space = "~tracker",
      std::chrono::duration<double> timeout = std::chrono::milliseconds(500))
      : BaseTracker(std::move(std::unique_ptr<TrackingStrategy>(
            new ClosestTrackingStrategy(default_num_retries_)))),
        nh_(name_space),
        alvar_sub_(nh_.subscribe("ar_pose_marker", 1,
                                 &AlvarTracker::markerCallback, this)),
        timeout_(timeout) {}
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
  * @brief Get the ROS time stamp of the current tracking vectors
  */
  virtual ros::Time getROSTrackingTime();

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
  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers &marker_msg);

  /**
  * @brief ROS node handle for communication
  */
  ros::NodeHandle nh_;
  /**
  * @brief Marker subscriber
  */
  ros::Subscriber alvar_sub_;
  /**
  * @brief Last time we received a non-empty Alvar message
  */
  Atomic<ros::Time> last_valid_time_;
  /**
  * @brief Last time we received a non-empty Alvar message
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
