#pragma once

/**
 * @brief Controls for a quadrotor given by a backstepping controller
 */
struct QrotorBSControl {
  double thrust_ddot;
  tf::Vector3 torque;
};
