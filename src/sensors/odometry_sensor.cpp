#include "aerial_autonomy/sensors/odometry_sensor.h"
#include <glog/logging.h>

OdomSensor::OdomSensor(OdomSensorConfig config)
    : nh_(config.ros_sensor_config().name_space()),
      pose_sub_(nh_.subscribe(
          config.ros_sensor_config().topic(), 1,
          &OdomSensor::odomCallback, this,
          ros::TransportHints().unreliable().reliable().tcpNoDelay())),
      config_(config) {}

std::pair<tf::StampedTransform, tf::Vector3>
OdomSensor::getSensorData() {
  return std::make_pair(tf::StampedTransform(pose_), tf::Vector3(velocity_));
}

// Take in odometry message and determine the pose as a stamped transform and velocity
void OdomSensor::odomCallback(
    const nav_msgs::OdometryConstPtr &odom_input) {

  if (isnan(odom_input->pose.pose.position.x) ||
      isnan(odom_input->pose.pose.position.y) ||
      isnan(odom_input->pose.pose.position.z) ||
      isnan(odom_input->pose.pose.orientation.x) ||
      isnan(odom_input->pose.pose.orientation.y) ||
      isnan(odom_input->pose.pose.orientation.z) ||
      isnan(odom_input->pose.pose.orientation.w))
  {
    return;
  }

  tf::Pose pose_out;
  tf::poseMsgToTF(odom_input->pose.pose, pose_out);
  const ros::Time stamp = odom_input->header.stamp;
  const std::string frame_id = odom_input->header.frame_id;
  const std::string child_frame_id = odom_input->child_frame_id;

  tf::StampedTransform pose_stamped_out(pose_out, stamp, frame_id, child_frame_id); 
  pose_ = pose_stamped_out;

  tf::Vector3 velocity(odom_input->twist.twist.linear.x, odom_input->twist.twist.linear.y, odom_input->twist.twist.linear.z);
  velocity_ = velocity;

  if (config_.publish_tf()) {
    // Publish tf 
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_input->header.stamp;
    tf_msg.header.frame_id = odom_input->header.frame_id;
    tf_msg.child_frame_id = config_.body_frame();
    tf::transformTFToMsg(pose_out, tf_msg.transform);
    br.sendTransform(tf_msg);
  }
}

SensorStatus OdomSensor::getSensorStatus() {
  tf::StampedTransform pose = pose_;
  ros::Duration duration_since_last_message = (ros::Time::now() - pose.stamp_);
  if (duration_since_last_message.toSec() >
      config_.ros_sensor_config().timeout()) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}