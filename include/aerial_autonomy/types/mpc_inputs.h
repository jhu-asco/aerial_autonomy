#pragma once
#include "constraint.h"
#include <Eigen/Dense>
#include <vector>

/**
* @brief Inputs to MPC optimization
*
* @tparam StateT type of initial state
*/
template <class StateT> struct MPCInputs {
  /**
  * @brief Initial condition for optimization
  */
  StateT initial_state;
  /**
  * @brief Dynamic constraints for optimization
  */
  std::vector<Constraint> constraints;
  /**
  * @brief Static parameters for MPC optimization
  */
  Eigen::VectorXd parameters;
  /**
   * @brief time_since_goal
   *
   * The time difference between the start of the
   * goal reference trajectory and when the initial
   * state is measured
   */
  double time_since_goal;
};
