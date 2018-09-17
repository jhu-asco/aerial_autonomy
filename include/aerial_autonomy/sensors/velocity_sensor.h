#pragma once
#include "aerial_autonomy/sensors/ros_sensor.h"
#include "aerial_autonomy/types/velocity.h"
#include "ros_sensor_config.pb.h"
#include <aerial_autonomy/common/conversions.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

/**
* @brief ros based velocity sensor
*/
class VelocitySensor : public Sensor<Velocity> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param Config for velocity sensor
  */
  VelocitySensor(ROSSensorConfig config) : sensor_(config) {
    local_transform_ =
        conversions::protoTransformToTf(config.sensor_transform());
  }
  /**
  * @brief gives sensor data
  */
  Velocity getSensorData() {
    nav_msgs::Odometry msg = sensor_.getSensorData();
    Velocity velocity_sensor_data(msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z);
    return velocity_sensor_data;
  }
  /**
  * @brief give sensor data, transformed by the local transform.
  * Returns the velocity in the sensor origin frame of the robot center.
  */
  Velocity getTransformedSensorData() {
    nav_msgs::Odometry msg = sensor_.getSensorData();
    tf::Vector3 linear_velocity_data(msg.twist.twist.linear.x,
                                     msg.twist.twist.linear.y,
                                     msg.twist.twist.linear.z);
    tf::Vector3 angular_velocity_data(msg.twist.twist.angular.x,
                                      msg.twist.twist.angular.y,
                                      msg.twist.twist.angular.z);
    tf::Transform rotation_only_transform(local_transform_.getRotation());
    tf::Vector3 transform_translation(local_transform_.getOrigin());
    tf::Vector3 transformed_velocity =
        rotation_only_transform *
            angular_velocity_data.cross(transform_translation) +
        linear_velocity_data;
    return Velocity(transformed_velocity.getX(), transformed_velocity.getY(),
                    transformed_velocity.getZ());
  }
  /**
  * @brief gives sensor status
  */
  SensorStatus getSensorStatus() { return sensor_.getSensorStatus(); }

private:
  /*
  * @brief Underlying ROS sensor.  Listens to a ROS topic of Odometry messages,
  * which we use for velocity.
  */
  ROSSensor<nav_msgs::Odometry> sensor_;
  /*
  * @brief Local transform provided by the config, from sensor frame to robot
  * frame.
  */
  tf::Transform local_transform_;
};
