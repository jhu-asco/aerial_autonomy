#pragma once
#include "aerial_autonomy/types/sensor_status.h"
#include <aerial_autonomy/common/atomic.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/tf.h>
#include <memory>
#include "path_sensor_config.pb.h"
#include <Eigen/Dense>

/**
* @brief Path sensor from a ros topic
* returns a tuple: (time received, vector of tuples (position, RPY, velocity, angular velocity))
*
* TODO: Merge in ROS sensor abstract class, then make this a child class of it
*
*/

//                            Position         Roll,Pitch,Yaw,  Velocity,        RPY Rates        Acceleration
using PathPointT = std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>;
using PathReturnT = std::tuple<double,double,std::vector<PathPointT>>;

class PathSensor : public Sensor<PathReturnT> {
public:
  /**
  * @brief Constructor
  *
  * @params config class
  */
PathSensor(PathSensorConfig config)
  : nh_(config.ros_sensor_config().name_space()),
    path_sub_(nh_.subscribe(
        config.ros_sensor_config().topic(), 1, &PathSensor::pathCallback, this)),
    config_(config){}


//PathSensor::~PathSensor() {}

  /**
  * @brief Destructor
  */
  //~PathSensor() {int i = 0; i++;};
  /**
  * @brief gets the latest sensor data
  *
  * @return most recent message
  */
  //PathReturnT getSensorData();
PathReturnT getSensorData(){ return message_;}
  /**
  * @brief gets the current status of the sensor
  *
  * @return sensor status
  */
  //SensorStatus getSensorStatus();
SensorStatus getSensorStatus() {
  double curr_time = ros::Time::now().toSec();
  PathReturnT curr_msg = message_;
  if ((curr_time - std::get<0>(curr_msg)) > config_.ros_sensor_config().timeout()) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}

  /**
  * @brief ROS callback function
  *
  * @param path_input input ROS message
  */
  //void pathCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg);
void pathCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg) {
  double time = msg->header.stamp.toSec();
  int N = msg->points.size();
  std::vector<PathPointT> path_vec(N);
  for (int ii = 0; ii < N; ++ii) {
    Eigen::Vector3d pos(msg->points[ii].positions[0],msg->points[ii].positions[1],msg->points[ii].positions[2]);
    Eigen::Vector3d rotLog(msg->points[ii].positions[3],msg->points[ii].positions[4],msg->points[ii].positions[5]);
    Eigen::Vector3d vel(msg->points[ii].velocities[0],msg->points[ii].velocities[1],msg->points[ii].velocities[2]);
    Eigen::Vector3d angvel(msg->points[ii].velocities[3],msg->points[ii].velocities[4],msg->points[ii].velocities[5]);
    Eigen::Vector3d acc(msg->points[ii].accelerations[0],msg->points[ii].accelerations[1],msg->points[ii].accelerations[2]);
    path_vec[ii] = std::make_tuple(pos,rotLog,vel,angvel,acc);
  }
  message_ = std::make_tuple(((double)time),config_.final_time(),path_vec);
}


private:
  ros::NodeHandle nh_;                             ///< Nodehandle
  ros::Subscriber path_sub_;                       ///< ros subscriber
  Atomic<PathReturnT> message_;              ///< latest pose
  PathSensorConfig config_;                        ///< Odom sensor config

};
