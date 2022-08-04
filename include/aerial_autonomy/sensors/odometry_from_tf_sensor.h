#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "odom_from_pose_sensor_config.pb.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

/**
* @brief Pose sensor from a tf
* \todo Gowtham Make this a generic templated ros sensor
*/
class OdomFromTFSensor
    : public Sensor<std::pair<tf::StampedTransform, tf::Vector3>> {
public:
  /**
  * @brief Constructor
  *
  * @param odom sensor config
  *
  */
  OdomFromTFSensor(OdomSensorConfig sensor_config);

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
  * @brief ROS query TF function
  */
  void getPose();

private:
  ros::NodeHandle nh_;                             ///< Nodehandle
  tf2_ros::Buffer buffer_;                         ///< ros tf2 buffer
  tf2_ros::TransformListener *listener_;   ///< ros tf2 listener
  boost::thread *tf_thread_;                       ///< thread for qurerying tf
  std::string tf_frame_;                           ///< tf frame of interest
  Atomic<tf::StampedTransform> pose_;              ///< latest pose
  Atomic<tf::Vector3> velocity_;                   ///< latest pose
  ExponentialFilter<tf::Vector3> velocity_filter_; ///< Filter velocity
  bool pose_initialized_;                          ///< Pose initialized
  OdomSensorConfig config_;                        ///< Odom sensor config
};
