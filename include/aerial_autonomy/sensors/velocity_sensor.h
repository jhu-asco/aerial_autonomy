#pragma once
#include "aerial_autonomy/sensors/ros_sensor.h"
#include "aerial_autonomy/types/velocity.h"
#include "ros_sensor_config.pb.h"
#include <aerial_autonomy/common/conversions.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
/**
* @brief ros based velocity sensor
*/
class VelocitySensor : public Sensor<Velocity> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param Config for velocity sensor
  */
  VelocitySensor(ROSSensorConfig config): sensor_(config) {
    //sensor_ = ROS_Sensor<nav_msgs::Odometry>(config);
    config_ = config;
  }
  /**
  * @brief gives sensor data
  */
  Velocity getSensorData() {
    nav_msgs::Odometry msg = sensor_.getSensorData();
    Velocity vel_sensor_data(msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.linear.z);
    return vel_sensor_data;
  }
  /**
  * @brief gives sensor status
  */
  SensorStatus getSensorStatus() {
    return sensor_.getSensorStatus();
  }

private:
  /*
  * @brief sensor
  */
  ROS_Sensor<nav_msgs::Odometry> sensor_;
  /*
  * @brief sensor config
  */
  ROSSensorConfig config_;
  /**
  * @brief sensor's origin in world frame
  */
  //tf::Transform sensor_world_tf_;
  /**
  * @brief variable to store sensor data
  */
  //Atomic<Velocity> sensor_data_;
};
