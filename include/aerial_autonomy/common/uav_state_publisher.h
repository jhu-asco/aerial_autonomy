#pragma once

#include <memory>

#include "aerial_autonomy/robot_systems/uav_system.h"
#include <ros/ros.h>

class UAVStatePublisher {
public:
  /**
  * @brief Constructor
  * @param drone_hardware Pointer to uav plugin
  */
  UAVStatePublisher(UAVSystem &uav_system);

  /**
  * @brief Publish uav state messages
  */
  void publish();

private:
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
  * @brief Pose publisher
  */
  ros::Publisher pose_pub_;
  /**
  * @brief Twist publisher
  */
  ros::Publisher twist_pub_;
};
