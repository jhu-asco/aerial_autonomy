#pragma once
#include "aerial_autonomy/sensors/ros_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "ros_sensor_config.pb.h"
#include <aerial_autonomy/common/conversions.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
/**
* @brief ros based velocity sensor
*/
class OdometrySensor : public Sensor<std::tuple<VelocityYawRate, PositionYaw>> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param Config for odometry sensor
  */
  OdometrySensor(ROSSensorConfig config) : ros_odom_sensor_(config) {
    local_transform_ =
        conversions::protoTransformToTf(config.sensor_transform());
  }
  /**
  * @brief gives sensor data
  */
  std::tuple<VelocityYawRate, PositionYaw> getSensorData() {
    nav_msg::Odometry msg = ros_odom_sensor_.getSensorData();
    // Map to velocity
    tf::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z);
    tf::Vector3 rate(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                     msg->twist.twist.angular.z);
    VelocityYawRate vyr(velocity.getX(), velocity.getY(), velocity.getZ(),
                        rate.getZ());
    // Map to position-yaw
    tf::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
    tf::Quaternion orientation(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Transform pyTransform(orientation, position);
    // Set the result
    PositionYaw py(
        pyTransform.getOrigin().getX(), pyTransform.getOrigin().getY(),
        pyTransform.getOrigin().getZ(), tf::getYaw(pyTransform.getRotation()));
    retVal = std::make_tuple(vyr, py);
    return retVal;
  }
  /**
  * @brief gives sensor data, transformed by the local transform (e.g. the
  * sensor to quad center transform)
  */
  std::tuple<VelocityYawRate, PositionYaw> getTransformedSensorData() {
    nav_msg::Odometry msg = ros_odom_sensor_.getSensorData();
    // Map the velocity and transform with only the rotation of the local
    // transform.
    tf::Transform rotation_only_transform(local_transform_.getRotation());
    tf::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z);
    tf::Vector3 rate(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                     msg->twist.twist.angular.z);
    velocity = velocity * rotation_only_transform;
    rate = velocity * rotation_only_transform;
    VelocityYawRate vyr(velocity.getX(), velocity.getY(), velocity.getZ(),
                        rate.getZ());
    // Map the position yaw and transform with the local transform.
    tf::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
    tf::Quaternion orientation(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Transform pyTransform(orientation, position);
    pyTransform = pyTransform * local_transform_;
    // Set the result
    PositionYaw py(
        pyTransform.getOrigin().getX(), pyTransform.getOrigin().getY(),
        pyTransform.getOrigin().getZ(), tf::getYaw(pyTransform.getRotation()));
    retVal = std::make_tuple(vyr, py);
    return retVal;
  }
  /**
  * @brief gives sensor status
  */
  SensorStatus getSensorStatus() { return ros_odom_sensor_.getSensorStatus(); }

private:
  /**
  * @brief ROS Topic listening sensor
  */
  ROSSensor<nav_msgs::Odometry> ros_odom_sensor_;
  /**
  * @brief The transform between the sensor and the robot frame.
  */
  tf::Transform local_transform_;
};
