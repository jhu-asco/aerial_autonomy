#include "aerial_autonomy/sensors/odometry_from_tf_sensor.h"
#include <glog/logging.h>
#include <boost/thread/thread.hpp>

OdomFromTFSensor::OdomFromTFSensor(OdomSensorConfig config)
    : nh_(config.ros_sensor_config().name_space()),
      tf_frame_(config.ros_sensor_config().topic()),
      velocity_filter_(config.velocity_filter_gain()), pose_initialized_(false),
      config_(config)
{
// Spawn thread for tf
listener_ = new tf2_ros::TransformListener(buffer_);
tf_thread_ = new boost::thread(boost::bind(&OdomFromTFSensor::getPose, this));
}

std::pair<tf::StampedTransform, tf::Vector3>
OdomFromTFSensor::getSensorData() {
  return std::make_pair(tf::StampedTransform(pose_), tf::Vector3(velocity_));
}

void OdomFromTFSensor::getPose() {
  ros::Rate rate(120);

  while (nh_.ok())
  {
    geometry_msgs::TransformStamped pose_input;
    try{
      pose_input = buffer_.lookupTransform("world", tf_frame_,
                                ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    if (isnan(pose_input.transform.translation.x) ||
        isnan(pose_input.transform.translation.y) ||
        isnan(pose_input.transform.translation.z) ||
        isnan(pose_input.transform.rotation.x) ||
        isnan(pose_input.transform.rotation.y) ||
        isnan(pose_input.transform.rotation.z) ||
        isnan(pose_input.transform.rotation.w))
    {
      continue;
    }

    tf::StampedTransform pose_out;
    tf::transformStampedMsgToTF(pose_input, pose_out);
    tf::StampedTransform previous_pose = pose_;
    // Velocity
    if (pose_initialized_) {
      double tdiff = (pose_out.stamp_ - previous_pose.stamp_).toSec();
      if (tdiff < 1e-2) {
        LOG(WARNING) << "Tdiff too small: " << tdiff;
        // tdiff = 1e-3;
        continue;
      }
      else if (tdiff >= 0.08) {
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

    rate.sleep();
  }

}

SensorStatus OdomFromTFSensor::getSensorStatus() {
  tf::StampedTransform pose = pose_;
  ros::Duration duration_since_last_message = (ros::Time::now() - pose.stamp_);
  if (duration_since_last_message.toSec() >
      config_.ros_sensor_config().timeout()) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}
