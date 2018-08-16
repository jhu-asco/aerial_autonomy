#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/sensors/ros_sensor.h"
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>

/**
* @brief Pose sensor from a ros topic
* \todo Gowtham Make this a generic templated ros sensor
*/
class PoseSensor : public Sensor<tf::StampedTransform> {
public:
  /**
  * @brief Constructor
  *
  * @param pose_topic ros topic name
  * @param validity_buffer timeout for messages
  * @param ns Name space for internal node handle
  *
  */
  PoseSensor(std::string pose_topic, ros::Duration validity_buffer,
             std::string ns = "~pose");

  /**
  * @brief  get the latest sensor measurement
  *
  * @return sensor measurement
  */
  tf::StampedTransform getSensorData();

  /**
  * @brief Get the status of the sensor
  *
  * @return sensor status
  */
  SensorStatus getSensorStatus();

  /**
  * @brief ROS callback function
  *
  * @param pose_input input ROS message
  */

private:
  ROS_Sensor<geometry_msgs::TransformStamped> *sensor_
};
