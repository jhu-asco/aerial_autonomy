#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include <ros/ros.h>
#include <tf/tf.h>

/**
* @brief Pose sensor from a ros topic
*/
class PoseSensor : public Sensor<tf::StampedTransform> {
public:
  PoseSensor(std::string pose_topic, ros::Duration validity_buffer,
             std::string ns = "~pose");

  tf::StampedTransform getSensorData();

  SensorStatus getSensorStatus();

  void poseCallback(const geometry_msgs::TransformStampedConstPtr &pose_input);

private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  Atomic<tf::StampedTransform> pose_;
  ros::Duration validity_buffer_;
};
