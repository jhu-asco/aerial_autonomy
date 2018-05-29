#pragma once
#include <tf/tf.h>

/**
 * @brief Controls for a quadrotor given by a backstepping controller
 */
struct QrotorBacksteppingControl {
  /**
  * @brief Second time dertivative of thrust
  */
  double thrust_ddot;
  /**
  * @brief Body torques
  */
  tf::Vector3 torque;
};
