#pragma once
#include "base_sensor.h"
#include <ros/ros.h>
#include "ros_sensor_config.pb.h"

/**
* @brief Base class for ROS sensors
*
* Subclass provides functionality to update sensor data
* and status
*/
template <class SensorDataT> 
class ROS_Sensor: public Sensor<SensorDataT> {
public:
  /**
  * @brief Constructor
  */
  ROS_Sensor(ROSSensorConfig config): config_(config) {
    VLOG(2) << "Initializing ROS Sensor";
    sub_ = nh_.subscribe(config.topic(), 1, &ROS_Sensor::callback, this);
    last_msg_time_ = ros::Time::now();
  }
  /**
  * @brief gets the latest sensor data
  */
  SensorDataT getSensorData() {
    SensorDataT retVal = sensor_data_;
    return retVal;
  }
  /**
  * @brief gets the current status of the sensor
  */
  SensorStatus getSensorStatus() {
    SensorStatus retVal;
    ros::Time last_msg_time = last_msg_time_;
    if ((ros::Time::now() - last_msg_time).toSec() > config_.timeout())
      retVal = SensorStatus::INVALID;
    else
      retVal = SensorStatus::VALID;
    return retVal;
  }

private:
  //Only works if SensorDataT is the proper ROS msg type
  void callback(const typename SensorDataT::ConstPtr msg) {
    ros::Time last_msg_time = msg->header.stamp;
    last_msg_time_ = last_msg_time;
    sensor_data_ = *msg;
  }
  ROSSensorConfig config_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  Atomic<ros::Time> last_msg_time_;
  Atomic<SensorDataT> sensor_data_;
};
