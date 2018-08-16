#include "aerial_autonomy/sensors/pose_sensor.h"

PoseSensor::PoseSensor(ROSSensorConfig config) : sensor_(config) {}

tf::StampedTransform PoseSensor::getSensorData() {
    const geometry_msgs::TransformStampedConstPtr &pose_input) {
      geometry_msgs::TransformStamped msg = sensor_->getSensorData();
      tf::StampedTransform pose_out;
      tf::transformStampedMsgToTF(msg, pose_out);
      return pose_out;
    }

    SensorStatus PoseSensor::getSensorStatus() {
      return sensor_->getSensorStatus();
    }
