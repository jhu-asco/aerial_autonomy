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
class OdomSensor : public Sensor<std::tuple<VelocityYawRate, PositionYaw>> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param Config for odometry sensor
  */
  OdomSensor(ROSSensorConfig config) : config_(config) {
    // sensor_quad_tf_ =
    //    conversions::protoTransformToTf(config_.sensor_transform());
    sensor_ = ROS_Sensor<nav_msgs::Odometry>(config);
  }
  /**
  * @brief gives sensor data
  */
  std::tuple<VelocityYawRate, PositionYaw> getSensorData() {
    nav_msg::Odometry msg = sensor_.getSensorData();
    // Transform the velocity
    tf::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z);
    tf::Vector3 rate(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                     msg->twist.twist.angular.z);
    VelocityYawRate vyr(velocity.getX(), velocity.getY(), velocity.getZ(),
                        rate.getZ());
    // Transform the position-yaw
    tf::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
    tf::Quaternion orientation(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Transform pyTransform(orientation, position);
    // pyTransform = pyTransform * sensor_quad_tf_;
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
  SensorStatus getSensorStatus() { return sensor_.getSensorStatus(); }

private:
  /**
  * @brief sensor
  */
  ROS_Sensor<nav_msgs::Odometry> sensor_;
  /**
  * @brief sensor config
  */
  ROSSensorConfig config_;
  /**
  * @brief sensor's origin in world frame
  */
  // tf::Transform sensor_quad_tf_;
  /**
  * @brief variable to store sensor data
  */
  // Atomic<std::tuple<VelocityYawRate, PositionYaw>> sensor_data_;
};
