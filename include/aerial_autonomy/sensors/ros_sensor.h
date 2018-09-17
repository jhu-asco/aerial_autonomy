#pragma once
#include "base_sensor.h"
#include "ros_sensor_config.pb.h"
#include <ros/ros.h>

/**
* @brief Base class for ROS sensors
*
* Subclass provides functionality to update sensor data
* and status
*/
template <class SensorDataT> class ROSSensor : public Sensor<SensorDataT> {
public:
  /**
  * @brief Constructor
  */
  ROSSensor(ROSSensorConfig config) : config_(config) {
    sub_ = nh_.subscribe(config.topic(), 1, &ROSSensor::callback, this);
    last_msg_time_ = ros::Time::now();
  }
  /**
  * @brief gets the latest sensor data
  */
  SensorDataT getSensorData() { return SensorDataT(sensor_data_); }
  /**
  * @brief gets the latest sensor data
  */
  SensorDataT getTransformedSensorData() { return SensorDataT(sensor_data_); }
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
  /**
  * @brief Listen to the topic and set sensor_data_.  Only works if SensorDataT
  * is a ROS msg type
  */
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
