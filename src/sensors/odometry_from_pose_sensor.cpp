#include "aerial_autonomy/sensors/odometry_from_pose_sensor.h"
#include <glog/logging.h>
#include "aerial_autonomy/common/conversions.h"

OdomFromPoseSensor::OdomFromPoseSensor(OdomSensorConfig config)
    : nh_(config.ros_sensor_config().name_space()),
      pose_sub_(nh_.subscribe(
          config.ros_sensor_config().topic(), 1,
          &OdomFromPoseSensor::poseCallback, this,
          ros::TransportHints().unreliable().reliable().tcpNoDelay())),
      velocity_filter_(config.velocity_filter_gain()), pose_initialized_(false),
      config_(config), 
      odom_to_body_transform_(conversions::protoTransformToTf(config_.odom_to_body_transform())) {}

std::pair<tf::StampedTransform, tf::Vector3>
OdomFromPoseSensor::getSensorData() {
  return std::make_pair(tf::StampedTransform(pose_), tf::Vector3(velocity_));
}

void OdomFromPoseSensor::poseCallback(
    const geometry_msgs::TransformStampedConstPtr &pose_input) {
  
  if (isnan(pose_input->transform.translation.x) ||
      isnan(pose_input->transform.translation.y) ||
      isnan(pose_input->transform.translation.z) ||
      isnan(pose_input->transform.rotation.x) ||
      isnan(pose_input->transform.rotation.y) ||
      isnan(pose_input->transform.rotation.z) ||
      isnan(pose_input->transform.rotation.w))
  {
    return;
  }

  tf::StampedTransform pose_out;
  tf::transformStampedMsgToTF(*pose_input, pose_out);

  if (config_.transform_to_body()) {
    tf::Transform new_pose_out = pose_out;
    new_pose_out = new_pose_out * odom_to_body_transform_;
    pose_out.setData(new_pose_out);
  }

  tf::StampedTransform previous_pose = pose_;
  // Velocity
  if (pose_initialized_) {
    double tdiff = (pose_out.stamp_ - previous_pose.stamp_).toSec();
    if (tdiff < 1e-3) {
      tdiff = 1e-3;
    }
    if (tdiff >= config_.ros_sensor_config().tdiff_warning()) {
      LOG(WARNING) << "Tdiff too big: " << tdiff;
    }
    tf::Vector3 velocity =
        (pose_out.getOrigin() - previous_pose.getOrigin()) / tdiff;
    velocity_ = velocity_filter_.addAndFilter(velocity);
  } else {
    velocity_ = tf::Vector3(0, 0, 0);
    pose_initialized_ = true;
  }
  pose_ = pose_out;

  if (config_.publish_tf()) {
    // Publish tf 
    br.sendTransform(*pose_input);

    if (config_.transform_to_body()) {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = pose_input->header.stamp;
      tf_msg.header.frame_id = pose_input->header.frame_id;
      tf_msg.child_frame_id = config_.body_frame();
      tf::transformTFToMsg(pose_out, tf_msg.transform);
      br.sendTransform(tf_msg);
    }
  }
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
