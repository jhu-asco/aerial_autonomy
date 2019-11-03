#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "odom_from_pose_sensor_config.pb.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

/**
* @brief Odometry sensor from a ros topic
* \todo Gowtham Make this a generic templated ros sensor
*/
class OdomSensor : public Sensor<std::pair<tf::StampedTransform, tf::Vector3>> {
public:
  /**
  * @brief Constructor
  *
  * @param odom_topic ros topic name
  * @param validity_buffer timeout for messages
  * @param ns Name space for internal node handle
  *
  */
  OdomSensor(OdomSensorConfig sensor_config);

  /**
  * @brief  get the latest sensor measurement
  *
  * @return sensor measurement
  */
  std::pair<tf::StampedTransform, tf::Vector3> getSensorData();

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
  void odomCallback(const nav_msgs::OdometryConstPtr &odom_input);

private:
  ros::NodeHandle nh_;                ///< Nodehandle
  ros::Subscriber pose_sub_;          ///< ros subscriber
  Atomic<tf::StampedTransform> pose_; ///< latest pose
  Atomic<tf::Vector3> velocity_;      ///< latest pose
  ros::Duration validity_buffer_;     ///< timeout for messages
  OdomSensorConfig config_;           ///< Odom sensor config
};
