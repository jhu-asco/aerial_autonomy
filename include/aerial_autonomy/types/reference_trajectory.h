#pragma once
#include <vector>

/**
* @brief A trajectory containing controls, states and timestamps
*
* @tparam StateT The type of state stored along the trajectory
* @tparam ControlT The type of control along the trajectory
*/
template <class StateT, class ControlT> struct ReferenceTrajectory {
  /**
  * @brief Time stamps corresponding to states
  */
  std::vector<double> ts;
  /**
  * @brief The states along the trajectory
  */
  std::vector<StateT> states;
  /**
  * @brief Controls along trajectory has dimension
  * 1 less than states.
  */
  std::vector<ControlT> controls;
};
