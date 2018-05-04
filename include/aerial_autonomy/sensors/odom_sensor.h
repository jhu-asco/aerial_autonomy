#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "odom_sensor_config.pb.h"
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
  OdomSensor(OdomSensorConfig config) : config_(config) {
    VLOG(2) << "Initialzing ROS Sensor";
    odom_sub_ = nh_.subscribe("/vins_estimator/odometry", 1,
                              &OdomSensor::odom_pose_Callback, this);
    sensor_quad_tf_ =
        conversions::protoTransformToTf(config_.sensor_transform());
    last_msg_time_ = ros::Time::now();
  }
  /**
  * @brief gives sensor data
  */
  std::tuple<VelocityYawRate, PositionYaw> getSensorData() {
    return sensor_data_;
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
  void odom_pose_Callback(const nav_msgs::Odometry::ConstPtr msg) {
    last_msg_time_ = msg->header.stamp;
    //Transform the velocity
    tf::Vector3 velocity(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    tf::Vector3 rate(msg->twist.twist.angular.x,
                     msg->twist.twist.angular.y,
                     msg->twist.twist.angular.z);
    //Set the result
    VelocityYawRate vyr(velocity.getX(),
                        velocity.getY(),
                        velocity.getZ(),
                        rate.getZ());
    //Transform the position-yaw
    tf::Vector3 position(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    tf::Quaternion orientation(msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z,
                               msg->pose.pose.orientation.w);
    tf::Transform pyTransform(orientation,position);
    pyTransform = pyTransform * sensor_quad_tf_;
    //Set the result
    PositionYaw py(pyTransform.getOrigin().getX(),
                   pyTransform.getOrigin().getY(),
                   pyTransform.getOrigin().getZ(),
                   tf::getYaw(pyTransform.getRotation()));
    //set the tuple to the sensor_data_
    sensor_data_ = std::make_tuple(vyr,py);
  }
  /**
  * @brief sensor config
  */
  OdomSensorConfig config_;
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
  tf::Transform sensor_quad_tf_;
  /**
  * @brief time of last msg recieved
  */
  Atomic<ros::Time> last_msg_time_;
  /**
  * @brief variable to store sensor data
  */
  Atomic<std::tuple<VelocityYawRate, PositionYaw>> sensor_data_;
};
