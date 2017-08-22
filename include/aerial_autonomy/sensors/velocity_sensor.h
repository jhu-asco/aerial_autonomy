#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "position_sensor_config.pb.h"
#include <parsernode/parser.h>

/**
* @brief ros based pose sensor that returns velocity data
*/
class VelocitySensor 
: public Sensor<VelocityYaw>
{
public:
  /**
  * 
  * @brief Constructor
  *
  * @param nodehandle for ros stuff
  *
  */
  VelocitySensor(parsernode::Parser &drone_hardware) : 
  config_(PositionSensorConfig()),
  drone_hardware_(drone_hardware)
  {
    pose_sub_ = nh_.subscribe("/pose", 1000, &VelocitySensor::poseCallback, this);
    
    sensor_tf.setOrigin(tf::Vector3(
      config_.sensor_tx(),
      config_.sensor_ty(),
      config_.sensor_tz()));

    sensor_tf.setRotation(tf::createQuaternionFromRPY(
      config_.sensor_r(),
      config_.sensor_p(),
      config_.sensor_y()));
  }
private:
  /**
  * @brief callback for pose sensor
  */
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
  {
    // Get gps data for checking if pose is diverging
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    double dt = config_.dt();

  // Convert to global frame
    tf::Vector3 pos = tf::Vector3(msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    tf::Vector3 global_pos = sensor_tf.inverse()*pos;

    if(abs(global_pos[0] - data.localpos.x) < config_.max_divergence() ||
      abs(global_pos[1]- data.localpos.y) < config_.max_divergence() ||
      abs(global_pos[2] - data.localpos.z) < config_.max_divergence())
    {
      bad_data_counter = 0;
    // Differentiate position to get velocity
      VelocityYaw vel_sensor_data;

      vel_sensor_data.x = (global_pos[0] - last_pos.x)/dt;
      vel_sensor_data.y = (global_pos[1] - last_pos.y)/dt;
      vel_sensor_data.z = (global_pos[2] - last_pos.z)/dt;

      last_pos.x = global_pos[0];
      last_pos.y = global_pos[1];
      last_pos.z = global_pos[2];

      // Convert to global frame
      tf::Quaternion q_s = tf::Quaternion(msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

      tf::Quaternion q = sensor_tf.inverse()*q_s;

      double r,p,y;
      tf::Matrix3x3 m(q);
      m.getRPY(r,p,y);

      vel_sensor_data.yaw = y;

      sensor_data_ = vel_sensor_data;
    }
    else
    {
      bad_data_counter++;
      if(bad_data_counter == 100)
        sensor_status_ = INVALID;
    }
  }
  /**
  * @ config for the position sensor
  */
  PositionSensorConfig config_;
  /**
  * @brief Quad hardware to compare data
  * to check validity
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief
  */
  ros::NodeHandle nh_;
  /**
  * @brief Subscriber for pose topic
  */
  ros::Subscriber pose_sub_;
  /**
  * @brief Transform between sensor and robot
  */
  tf::Transform sensor_tf;
  /**
  * @brief variable to store last pose
  */
  PositionYaw last_pos;
  /**
  * @brief counter for bad data. 
  * Sensor status set to invalid if counter goes beyond threshold
  */
  int bad_data_counter;
};