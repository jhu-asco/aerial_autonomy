#pragma once
#include <utility>

/**
* @brief An interface for retrieving states and controls from a trajectory
*
* @tparam StateT The type of state stored along the trajectory
* @tparam ControlT The type of control along the trajectory
*/
template <class StateT, class ControlT> class ReferenceTrajectory {
public:
  /**
  * @brief Gets the trajectory information at the specified time
  * @param t Time
  * @return Trajectory state and control
  */
  virtual std::pair<StateT, ControlT> atTime(double t) const = 0;
};
