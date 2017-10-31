#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include "aerial_autonomy/common/atomic.h"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/ros.h>

/**
 * @brief Provides transform to a tracked object based on output from alvar
 */
class AlvarTracker : public BaseTracker {
public:
  /**
  * @brief Constructor
  */
  AlvarTracker(std::string name_space = "~tracker")
      // \todo Matt Add timeout and num_retries as config
      : BaseTracker(new ClosestTrackingStrategy(25)),
        nh_(name_space),
        alvar_sub_(nh_.subscribe("ar_pose_marker", 1,
                                 &AlvarTracker::markerCallback, this)),
        timeout_(0.5) {}
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
  * @brief Stored tracking transforms
  */
  Atomic<std::unordered_map<uint32_t, tf::Transform>> object_poses_;
  /**
  * @brief Timeout for valid update
  */
  const double timeout_;
};
