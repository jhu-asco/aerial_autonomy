#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include "aerial_autonomy/common/atomic.h"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/ros.h>

/**
 * @brief Provides vector to a tracked object based on output from alvar
 */
class AlvarTracker : public BaseTracker {
public:
  /**
  * @brief Constructor
  * @param nh ROS node handle for comms
  */
  AlvarTracker(ros::NodeHandle &nh)
      : BaseTracker(new ClosestTrackingStrategy()), nh_(nh),
        alvar_sub_(nh_.subscribe("ar_pose_marker", 1,
                                 &AlvarTracker::markerCallback, this)) {}
  /**
   * @brief Get the tracking vectors
   * @param pos Returned tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVectors(std::unordered_map<uint32_t, Position> &pos);
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
  * @brief Last time we received an Alvar message
  */
  Atomic<ros::Time> last_update_time_;
  /**
  * @brief Stored tracking position
  */
  Atomic<std::unordered_map<uint32_t, Position>> object_positions_;
};
