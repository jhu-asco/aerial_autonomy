#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "velocity_sensor_config.pb.h"
#include <aerial_autonomy/common/math.h>
#include <geometry_msgs/PoseStamped.h>
#include <parsernode/parser.h>
#include <ros/ros.h>

/**
* @brief ros based pose sensor that returns velocity data
*/
class VelocitySensor : public Sensor<VelocityYaw> {
public:
  /**
  *
  * @brief Constructor
  *
  * @brief UAV hardware for getting gps data
  * \todo soham take a GPSSensor instead
  *
  * @param nodehandle for ros stuff
  *
  * @param Config for velocity sensor
  */
  VelocitySensor(parsernode::Parser &drone_hardware, ros::NodeHandle nh,
                 VelocitySensorConfig config)
      : Sensor(SensorStatus::INVALID), config_(config),
        drone_hardware_(drone_hardware), nh_(nh) {
    pose_sub_ = nh_.subscribe("pose", 1, &VelocitySensor::poseCallback, this);
    sensor_tf = math::getTransformFromVector(config_.sensor_transform());
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    tf::Vector3 quad_intial_pos =
        tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z);
    tf::Quaternion quad_intial_rot = tf::createQuaternionFromRPY(
        data.rpydata.r, data.rpydata.p, data.rpydata.y);

    quad_intial_tf.setOrigin(quad_intial_pos);
    quad_intial_tf.setRotation(quad_intial_rot);
  }

private:
  /**
  * @brief callback for pose sensor
  */
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr msg) {
    // Get gps data for checking if pose is diverging
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    // Convert to global frame
    tf::Vector3 pos = tf::Vector3(msg->pose.position.x, msg->pose.position.y,
                                  msg->pose.position.z);

    tf::Quaternion q_s =
        tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                       msg->pose.orientation.z, msg->pose.orientation.w);

    tf::Transform sensor_world_tf;
    sensor_world_tf.setOrigin(pos);
    sensor_world_tf.setRotation(q_s);

    tf::Transform global_tf =
        quad_intial_tf * sensor_tf * sensor_world_tf * sensor_tf.inverse();
    tf::Vector3 global_pos = global_tf.getOrigin();
    tf::Quaternion global_q = global_tf.getRotation();

    if (abs(global_pos[0] - data.localpos.x) < config_.max_divergence() &&
        abs(global_pos[1] - data.localpos.y) < config_.max_divergence() &&
        abs(global_pos[2] - data.localpos.z) < config_.max_divergence()) {
      if (sensor_status_ == SensorStatus::INVALID) {
        sensor_status_ = SensorStatus::VALID;
        last_pos = global_pos;
        // Differentiate position to get
      } else {
        ros::Time current_msg_time = msg->header.stamp;
        double dt = (current_msg_time - last_msg_time).toSec();
        if (dt < config_.min_timestep())
          dt = config_.min_timestep();

        VelocityYaw vel_sensor_data;
        vel_sensor_data.x = (global_pos[0] - last_pos.x) / dt;
        vel_sensor_data.y = (global_pos[1] - last_pos.y) / dt;
        vel_sensor_data.z = (global_pos[2] - last_pos.z) / dt;

        last_pos = global_pos;
        vel_sensor_data.yaw = tf::getYaw(global_q);
        sensor_data_ = vel_sensor_data;
      }
      last_msg_time = msg->header.stamp;
    } else {
      if ((msg->header.stamp - last_msg_time).toSec() >=
          config_.bad_data_timeout())
        sensor_status_ = SensorStatus::INVALID;
    }
  }
  /**
  * @ config for the position sensor
  */
  VelocitySensorConfig config_;
  /**
  * @brief Quad hardware to compare data
  * to check validity
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief nodehandle for ros stuff
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
  * @brief quad's initial transform in world (NED) frame
  */
  tf::Transform quad_intial_tf;
  /**
  * @brief variable to store last pose
  */
  tf::Vector3 last_pos;
  /**
  * @brief time of last msg
  */
  ros::Time last_msg_time;
};
