#pragma once
#include "aerial_autonomy/types/reference_trajectory.h"
#include <vector>

/**
* @brief An interface for retrieving states and controls from a trajectory
*
* @tparam StateT The type of state stored along the trajectory
* @tparam ControlT The type of control along the trajectory
*/
template <class StateT, class ControlT>
class DiscreteReferenceTrajectory
    : public ReferenceTrajectory<StateT, ControlT> {
public:
  // TODO add constructor that verifies sizes of ts, states, controls are
  // correct
  /**
  * @brief Gets the trajectory information at the specified time
  * @param t Time
  * @return Trajectory state and control
  */
  virtual std::pair<StateT, ControlT> atTime(double t) const = 0;
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
