#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/velocity.h"
#include "velocity_sensor_config.pb.h"
#include <aerial_autonomy/common/conversions.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
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
  VelocitySensor(VelocitySensorConfig config) : config_(config) {
    VLOG(2) << "Initialzing ROS Sensor";
    odom_sub_ = nh_.subscribe("/velocity_sensor/odometry", 1,
                              &VelocitySensor::odomCallback, this);
    sensor_world_tf_ =
        conversions::protoTransformToTf(config_.sensor_transform());
    last_msg_time_ = ros::Time::now();
  }
  /**
  * @brief gives sensor data
  */
  Velocity getSensorData() {
    Velocity sensor_data = sensor_data_;
    return sensor_data;
  }
  /**
  * @brief gives sensor status
  */
  SensorStatus getSensorStatus() {
    SensorStatus sensor_status;
    ros::Time last_msg_time = last_msg_time_;
    if ((ros::Time::now() - last_msg_time).toSec() > config_.timeout())
      sensor_status = SensorStatus::INVALID;
    else
      sensor_status = SensorStatus::VALID;

    return sensor_status;
  }

private:
  /**
  * @brief callback for pose sensor
  */
  void odomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    ros::Time last_msg_time = msg->header.stamp;
    last_msg_time_ = last_msg_time;
    tf::Vector3 global_vel =
        sensor_world_tf_ * tf::Vector3(msg->twist.twist.linear.x,
                                       msg->twist.twist.linear.y,
                                       msg->twist.twist.linear.z);
    Velocity vel_sensor_data(global_vel[0], global_vel[1], global_vel[2]);
    sensor_data_ = vel_sensor_data;
  }
  /**
  * @brief sensor config
  */
  VelocitySensorConfig config_;
  /**
  * @brief nodehandle for ros stuff
  */
  ros::NodeHandle nh_;
  /**
  * @brief Subscriber for odometry topic
  */
  ros::Subscriber odom_sub_;
  /**
  * @brief sensor's origin in world frame
  */
  tf::Transform sensor_world_tf_;
  /**
  * @brief time of last msg recieved
  */
  Atomic<ros::Time> last_msg_time_;
  /**
  * @brief variable to store sensor data
  */
  Atomic<Velocity> sensor_data_;
};
