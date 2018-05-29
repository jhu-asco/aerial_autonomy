#pragma once
#include <vector>

/**
* @brief The arm state
*/
struct ArmState {
  /**
  * @brief Time corresponding to arm state
  */
  double t;
  /**
  * @brief Joint angles in rad
  */
  std::vector<double> joint_angles;
  /**
  * @brief Joint velocities in rad/s
  */
  std::vector<double> joint_velocities;
  /**
  * @brief Desired joint angles in radians
  */
  std::vector<double> desired_joint_angles;
};
