#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw.h"
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
    sensor_tf_ = math::getTransformFromVector(config_.sensor_transform());
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    quad_initial_tf_ = tf::Transform(
        tf::createQuaternionFromRPY(data.rpydata.x, data.rpydata.y,
                                    data.rpydata.z),
        tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z));
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
    tf::Transform sensor_world_tf(
        tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                       msg->pose.orientation.z, msg->pose.orientation.w),
        tf::Vector3(msg->pose.position.x, msg->pose.position.y,
                    msg->pose.position.z));

    // quad_initial_tf*sensor_tf_ is the sensor's origin in world frame
    // sensor_world_tf is sensor's current tf in sensor's origin
    // sensor_tf_.inverse() is quad in sensor's current frame

    tf::Transform global_tf =
        quad_initial_tf_ * sensor_tf_ * sensor_world_tf * sensor_tf_.inverse();
    tf::Vector3 global_pos = global_tf.getOrigin();
    tf::Quaternion global_q = global_tf.getRotation();

    if (abs(global_pos[0] - data.localpos.x) < config_.max_divergence() &&
        abs(global_pos[1] - data.localpos.y) < config_.max_divergence() &&
        abs(global_pos[2] - data.localpos.z) < config_.max_divergence()) {
      if (sensor_status_ == SensorStatus::INVALID) {
        SensorStatus sensor_status(SensorStatus::VALID);
        sensor_status_ = sensor_status;
        last_pos_ = global_pos;
        // Differentiate position to get
      } else {
        ros::Time current_msg_time = msg->header.stamp;
        double dt = (current_msg_time - last_msg_time_).toSec();
        if (dt < config_.min_timestep())
          dt = config_.min_timestep();

        VelocityYaw vel_sensor_data;
        vel_sensor_data.x = (global_pos[0] - last_pos_[0]) / dt;
        vel_sensor_data.y = (global_pos[1] - last_pos_[1]) / dt;
        vel_sensor_data.z = (global_pos[2] - last_pos_[2]) / dt;

        last_pos_ = global_pos;
        vel_sensor_data.yaw = tf::getYaw(global_q);
        sensor_data_ = vel_sensor_data;
      }
      last_msg_time_ = msg->header.stamp;
    } else {
      if ((msg->header.stamp - last_msg_time_).toSec() >=
          config_.bad_data_timeout()) {
        SensorStatus sensor_status(SensorStatus::INVALID);
        sensor_status_ = sensor_status;
      }
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
  tf::Transform sensor_tf_;
  /**
  * @brief quad's initial transform in world (NED) frame
  */
  tf::Transform quad_initial_tf_;
  /**
  * @brief variable to store last pose
  */
  tf::Vector3 last_pos_;
  /**
  * @brief time of last msg
  */
  ros::Time last_msg_time_;
};
