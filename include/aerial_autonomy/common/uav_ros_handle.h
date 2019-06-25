#pragma once

#include "aerial_autonomy/robot_systems/uav_system.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class UAVRosHandle {
public:
  /**
  * @brief Constructor
  * @param uav_system UAV system
  */
  UAVRosHandle(UAVSystem &uav_system);

  /**
  * @brief Publish uav state messages
  */
  void publish();

private:
  /**
  * @brief Update UAVSystem reference config from message
  * @param message Config message
  */
  void refControllerConfigCallback(std_msgs::Float32MultiArray message);
  /**
  * @brief Publish uav pose
  * @param data UAV state data
  */
  void publishPose(const parsernode::common::quaddata &data);
  /*
  * @brief Publish uav twist
  * @param data UAV state data
  */
  void publishTwist(const parsernode::common::quaddata &data);
  /**
  * @brief Node handle used for publishing
  */
  ros::NodeHandle nh_;

  /**
  * @brief UAV system
  */
  UAVSystem &uav_system_;

  /**
  * @brief Subscribes to reference controller config
  */
  ros::Subscriber ref_controller_config_sub_;

  /**
  * @brief Pose publisher
  */
  ros::Publisher pose_pub_;
  /**
  * @brief Twist publisher
  */
  ros::Publisher twist_pub_;
};
