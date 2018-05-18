#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

/**
* @brief Helper class that records mocap data to a log file
*/
class MocapLogger {
public:
  /**
  * @brief Constructor
  *
  * Adds a header to log file
  */
  MocapLogger();

protected:
  /**
  * @brief subscriber function
  *
  * @param data pose message obtained from mocap
  */
  void logData(const geometry_msgs::TransformStampedConstPtr data);
  /**
  * @brief Nodehandle to create subscriber
  */
  ros::NodeHandle nh_;
  /**
  * @brief Subscriber to get data from ros topic
  */
  ros::Subscriber mocap_sub_;
};
