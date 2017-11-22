#pragma once

#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

class GuidanceObstacleTracker : public BaseTracker {
public:
  GuidanceObstacleTracker(std::string name_space = "~tracker",
                          double msg_timeout = 0.2)
      : BaseTracker(
            std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy())),
        nh_(name_space), guidance_obstacle_sub_(nh_.subscribe(
                             "/guidance/obstacle_distance", 1,
                             &GuidanceObstacleTracker::guidanceCallback, this)),
        msg_timeout_(msg_timeout) {}
  /**
   * @brief Get the tracking vectors
   * @param pos Returned map of tracking vectors
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
  * @brief Check whether ROS topics are connected
  * @return True if connected, false otherwise
  */
  bool isConnected();

private:
  void guidanceCallback(const sensor_msgs::LaserScan &obstacle_msg);

  ros::NodeHandle nh_;
  ros::Subscriber guidance_obstacle_sub_;

  Atomic<std::vector<tf::Transform>> obstacle_transforms_;
  Atomic<ros::Time> last_obstacle_time_;
  const double msg_timeout_;
};
