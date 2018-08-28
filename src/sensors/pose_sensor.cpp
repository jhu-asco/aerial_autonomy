#include "aerial_autonomy/sensors/pose_sensor.h"

PoseSensor::PoseSensor(std::string pose_topic, ros::Duration validity_buffer,
                       std::string ns)
    : nh_(ns), pose_sub_(nh_.subscribe(
                   pose_topic, 1, &PoseSensor::poseCallback, this,
                   ros::TransportHints().unreliable().reliable().tcpNoDelay())),
      validity_buffer_(validity_buffer) {}

tf::StampedTransform PoseSensor::getSensorData() {
  return tf::StampedTransform(pose_);
}

void PoseSensor::poseCallback(
    const geometry_msgs::TransformStampedConstPtr &pose_input) {
  tf::StampedTransform pose_out;
  tf::transformStampedMsgToTF(*pose_input, pose_out);
  pose_ = pose_out;
}

SensorStatus PoseSensor::getSensorStatus() {
  tf::StampedTransform pose = pose_;
  ros::Duration duration_since_last_message = (ros::Time::now() - pose.stamp_);
  if (duration_since_last_message > validity_buffer_) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}
