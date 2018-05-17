#pragma once
#include <tf/tf.h>

/**
 * @brief State of a quadrotor system with additional dynamic
 * compensation state variables for use with a backstepping controller
 */
struct QrotorBSState {
  tf::Transform pose;
  tf::Vector3 v;
  tf::Vector3 w;
  double thrust;
  double thrust_dot;
};
