#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/sensors/ros_sensor.h"
#include <aerial_autonomy/common/conversions.h>
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
  * @param config configuration for the sensor.
  *
  */
  PoseSensor(ROSSensorConfig config);

  /**
  * @brief  get the latest sensor measurement
  *
  * @return sensor measurement
  */
  tf::StampedTransform getSensorData();
  /**
  * @brief  get the latest sensor measurement
  *
  * @return sensor measurement
  */
  tf::StampedTransform getTransformedSensorData();

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
  ROSSensor<geometry_msgs::TransformStamped> sensor_;
  tf::Transform local_transform_;
};
