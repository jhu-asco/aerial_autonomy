#pragma once
#include <tf/tf.h>

/**
* @brief The quadrotor state
*/
struct QuadState {
  /**
  * @brief The pose of quadrotor
  */
  tf::Transform pose;
  /**
  * @brief The velocity in inertial frame
  */
  tf::Vector3 velocity;
  /**
  * @brief The derivative of roll, pitch, yaw
  */
  tf::Vector3 rpydot;
  /**
  * @brief The desired roll, pitch, yaw
  */
  tf::Vector3 rpyd;
  /**
  * @brief The gain between thrust command and body z acceleration
  */
  double kt;
};
