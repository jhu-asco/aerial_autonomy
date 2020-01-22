#include "aerial_autonomy/sensors/odometry_from_pose_sensor.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/log/log.h"
#include <glog/logging.h>

OdomFromPoseSensor::OdomFromPoseSensor(OdomSensorConfig config)
    : nh_(config.ros_sensor_config().name_space()),
      pose_sub_(nh_.subscribe(
          config.ros_sensor_config().topic(), 1,
          &OdomFromPoseSensor::poseCallback, this,
          ros::TransportHints().unreliable().reliable().tcpNoDelay())),
      velocity_filter_(config.velocity_filter_gain()), pose_initialized_(false),
      config_(config) {
  DATA_HEADER("odometry_from_pose_sensor") << "pos_x"
                                           << "pos_y"
                                           << "pos_z"
                                           << "vel_x"
                                           << "vel_y"
                                           << "vel_z"
                                           << "roll"
                                           << "pitch"
                                           << "yaw" << DataStream::endl;
}

std::pair<tf::StampedTransform, tf::Vector3>
OdomFromPoseSensor::getSensorData() {
  return std::make_pair(tf::StampedTransform(pose_), tf::Vector3(velocity_));
}

void OdomFromPoseSensor::poseCallback(
    const geometry_msgs::TransformStampedConstPtr &pose_input) {
  tf::StampedTransform pose_out;
  tf::transformStampedMsgToTF(*pose_input, pose_out);
  tf::StampedTransform previous_pose = pose_;
  auto rpy = conversions::transformTfToRPY(pose_out);
  if (pose_initialized_) {
    double tdiff = (pose_out.stamp_ - previous_pose.stamp_).toSec();
    if (tdiff < 1e-3) {
      tdiff = 1e-3;
    }
    if (tdiff >= 0.05) {
      LOG(WARNING) << "Tdiff too big: " << tdiff;
    }
    tf::Vector3 velocity =
        (pose_out.getOrigin() - previous_pose.getOrigin()) / tdiff;
    velocity_ = velocity_filter_.addAndFilter(velocity);

    // If yaw rate is above limit, assume it was a sensor glitch and keep last solution
    /*
    auto last_rpy = conversions::transformTfToRPY(previous_pose); 
    if (fabs(rpy(2) - last_rpy(2))  / tdiff > config_.yaw_rate_limit()) {
      LOG(WARNING) << "Filtering out last pose. Yaw: " << fabs(rpy(2) - last_rpy(2))  / tdiff;
      auto stamp = pose_out.stamp_;
      pose_out = previous_pose;
      pose_out.stamp_ = stamp;
      rpy = last_rpy;
    }
    */
  } else {
    velocity_ = tf::Vector3(0, 0, 0);
    pose_initialized_ = true;
  }
  
  pose_ = pose_out;
  
  auto trans = pose_out.getOrigin();
  DATA_LOG("odometry_from_pose_sensor")
      << trans.x() << trans.y() << trans.z() << velocity_.get().x()
      << velocity_.get().y() << velocity_.get().z() << rpy.x() << rpy.y()
      << rpy.z() << DataStream::endl;
}

SensorStatus OdomFromPoseSensor::getSensorStatus() {
  tf::StampedTransform pose = pose_;
  ros::Duration duration_since_last_message = (ros::Time::now() - pose.stamp_);
  if (duration_since_last_message.toSec() >
      config_.ros_sensor_config().timeout()) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}
