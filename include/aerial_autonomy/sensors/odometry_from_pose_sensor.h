#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include "aerial_autonomy/sensors/transformed_sensor.h"
#include "odom_from_pose_sensor_config.pb.h"
#include <ros/ros.h>
#include <tf/tf.h>

/**
* @brief Pose sensor from a ros topic
* \todo Gowtham Make this a generic templated ros sensor
*/
class OdomFromPoseSensor
    : public TransformedSensor<std::pair<tf::StampedTransform, tf::Vector3>> {
public:
  /**
  * @brief Constructor
  *
  * @param odom sensor config
  *
  */
  OdomFromPoseSensor(OdomSensorConfig sensor_config);

  /**
  * @brief  get the latest sensor measurement
  *
  * @return sensor measurement
  */
  std::pair<tf::StampedTransform, tf::Vector3> getSensorData();
  std::pair<tf::StampedTransform, tf::Vector3> getTransformedSensorData() {
    return getSensorData();
  }

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
  void poseCallback(const geometry_msgs::TransformStampedConstPtr &pose_input);

private:
  ros::NodeHandle nh_;                             ///< Nodehandle
  ros::Subscriber pose_sub_;                       ///< ros subscriber
  Atomic<tf::StampedTransform> pose_;              ///< latest pose
  Atomic<tf::Vector3> velocity_;                   ///< latest pose
  ExponentialFilter<tf::Vector3> velocity_filter_; ///< Filter velocity
  bool pose_initialized_;                          ///< Pose initialized
  OdomSensorConfig config_;                        ///< Odom sensor config
};
