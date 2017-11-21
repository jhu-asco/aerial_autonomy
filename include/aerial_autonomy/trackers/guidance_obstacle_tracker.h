#pragma once

#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

class GuidanceObstacleTracker : public BaseTracker {
public:
  GuidanceObstacleTracker(std::string name_space = "~tracker")
      : BaseTracker(
            std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy())),
        nh_(name_space),
        guidance_obstacle_sub_(
            nh_.subscribe("/guidance/obstacle_distance", 1,
                          &GuidanceObstacleTracker::guidanceCallback, this)) {}
  /**
   * @brief Get the tracking vectors
   * @param pos Returned map of tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool
  getTrackingVectors(std::unordered_map<uint32_t, tf::Transform> &pos) = 0;
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid() = 0;

private:
  void guidanceCallback(const sensor_msgs::LaserScan &obstacle_msg);

  ros::NodeHandle nh_;
  ros::Subscriber guidance_obstacle_sub_;

  Atomic<std::vector<tf::Transform>> obstacle_transforms_;
  Atomic<ros::Time> last_obstacle_time_;
  const double msg_timeout_ = 0.2;
};
