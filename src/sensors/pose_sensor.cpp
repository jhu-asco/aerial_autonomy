#include "aerial_autonomy/sensors/pose_sensor.h"

PoseSensor::PoseSensor(ROSSensorConfig config) : sensor_(config) {
  local_transform_ = conversions::protoTransformToTf(config.sensor_transform());
}

tf::StampedTransform PoseSensor::getSensorData() {
  geometry_msgs::TransformStamped msg = sensor_.getSensorData();
  tf::StampedTransform pose_out;
  tf::transformStampedMsgToTF(msg, pose_out);
  return pose_out;
}

tf::StampedTransform PoseSensor::getTransformedSensorData() {
  tf::StampedTransform raw_data = getSensorData();
  tf::Transform transformed_data = local_transform_ * raw_data;
  return tf::StampedTransform(transformed_data, raw_data.stamp_,
                              raw_data.frame_id_, raw_data.child_frame_id_);
}

SensorStatus PoseSensor::getSensorStatus() { return sensor_.getSensorStatus(); }
