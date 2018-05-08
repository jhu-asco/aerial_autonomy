#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "odom_sensor_config.pb.h"
#include <aerial_autonomy/common/conversions.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
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
    odom_sub_ = nh_.subscribe("/vrpn_client/matrice/pose", 1,
                              &OdomSensor::odom_pose_Callback, this);
    sensor_quad_tf_ =
        conversions::protoTransformToTf(config_.sensor_transform());
    last_msg_time_ = ros::Time::now();
    sensor_data_ = std::make_tuple(VelocityYawRate(),PositionYaw());
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
  void odom_pose_Callback(const geometry_msgs::TransformStamped::ConstPtr msg) {
    //Transform the position-yaw
    tf::Vector3 position(msg->transform.translation.x,
                        msg->transform.translation.y,
                        msg->transform.translation.z);
    tf::Quaternion orientation(msg->transform.rotation.x,
                               msg->transform.rotation.y,
                               msg->transform.rotation.z,
                               msg->transform.rotation.w);
    tf::Transform pyTransform(orientation,position);
    pyTransform = pyTransform * sensor_quad_tf_;
    //Set the result
    PositionYaw py(pyTransform.getOrigin().getX(),
                   pyTransform.getOrigin().getY(),
                   pyTransform.getOrigin().getZ(),
                   tf::getYaw(pyTransform.getRotation()));
    //Find the difference velocity
    std::tuple<VelocityYawRate,PositionYaw> sensor_data = sensor_data_;
    PositionYaw deltaPosYaw = py - std::get<1>(sensor_data);
    double deltaT = (msg->header.stamp - last_msg_time_).toSec();
    deltaPosYaw = deltaPosYaw * (1/deltaT);
    VelocityYawRate vyr(deltaPosYaw.x,
                        deltaPosYaw.y,
                        deltaPosYaw.z,
                        deltaPosYaw.yaw);
    //Filter the velocity with the last velocity
    vyr = vyr * alpha + std::get<0>(sensor_data) * (1 - alpha);
    //set the tuple to the sensor_data_
    sensor_data_ = std::make_tuple(vyr,py);
    last_msg_time_ = msg->header.stamp;
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

  double alpha = 0.5;
};
