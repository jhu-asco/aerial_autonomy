#pragma once
#include <tf/tf.h>

/**
 * @brief State of a quadrotor system with additional dynamic
 * compensation state variables for use with a backstepping controller
 */
struct QrotorBacksteppingState {
  /**
  * @brief Default constructor
  */
  QrotorBacksteppingState()
      : pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)), v(0, 0, 0),
        w(0, 0, 0), thrust(0), thrust_dot(0) {}
  /**
  * @brief SE(3) pose
  */
  tf::Transform pose;
  /**
  * @brief Velocity
  */
  tf::Vector3 v;
  /**
  * @brief Angular velocity
  */
  tf::Vector3 w;
  /**
  * @brief Body-z thrust
  */
  double thrust;
  /**
  * @brief Time derivative of thrust
  */
  double thrust_dot;
};
