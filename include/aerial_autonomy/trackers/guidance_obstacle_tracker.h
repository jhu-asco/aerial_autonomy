#pragma once

#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

/**
* @brief This class subscribes to a ROS topic published by the Guidance ROS
* node.
* The topic publishes obstacle information, which is then stored by this class.
*/
class GuidanceObstacleTracker : public BaseTracker {
public:
  /**
  * @brief Constructor
  * @param name_space Namespace for internal ROS node
  * @param msg_timeout Duration of time until an obstacle message becomes old
  * and invalid
  */
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
  /**
  * @brief Callback for receiving obstacle messages from guidance.  Stores
  * obstacles in a vector for later retrieval
  * @param obstacle_msg Message containing obstacle information
  */
  void guidanceCallback(const sensor_msgs::LaserScan &obstacle_msg);

  /**
  * @brief Nodehandle for subscribing to Guidance node
  */
  ros::NodeHandle nh_;
  /**
  * @brief Subscriber to obstacle message topic
  */
  ros::Subscriber guidance_obstacle_sub_;
  /**
  * @brief Obstacle transforms from last received message
  */
  Atomic<std::vector<tf::Transform>> obstacle_transforms_;
  /**
  * @brief Last time subscriber received an obstacle message
  */
  Atomic<ros::Time> last_obstacle_time_;
  /**
  * @brief Time until an obstacle message becomes invalid
  */
  const double msg_timeout_;
};
