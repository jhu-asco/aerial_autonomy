#include "aerial_autonomy/sensors/mocap_sensor.h"

MocapSensor::MocapSensor(std::string mocap_topic, ros::Duration validity_buffer,
                         std::string ns)
    : nh_(ns), mocap_sub_(nh_.subscribe(mocap_topic, 1,
                                        &MocapSensor::mocapCallback, this)),
      validity_buffer_(validity_buffer) {}

tf::StampedTransform MocapSensor::getSensorData() {
  return tf::StampedTransform(pose_);
}

void MocapSensor::mocapCallback(
    const geometry_msgs::TransformStampedConstPtr &pose_input) {
  tf::StampedTransform pose_out;
  tf::transformStampedMsgToTF(*pose_input, pose_out);
  pose_ = pose_out;
}

SensorStatus MocapSensor::getSensorStatus() {
  tf::StampedTransform pose = pose_;
  ros::Duration duration_since_last_message = (ros::Time::now() - pose.stamp_);
  if (duration_since_last_message > validity_buffer_) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}
