#pragma once
#include "aerial_autonomy/types/discrete_reference_trajectory_closest.h"
#include <chrono>

/**
* @brief Select the control to be applied given the reference trajectory and
* current system state
*
* @tparam StateT The type of state
* @tparam ControlT The type of control
*/
template <class StateT, class ControlT> class AbstractControlSelector {
public:
  /**
  * @brief Select the control given reference trajectory and current state
  *
  * @param trajectory input reference trajectory
  * @param current_state Current system state
  * @param dt The time difference between the state0 in reference trajectory
  *            and the current state
  *
  * @return the control to be applied
  */
  virtual ControlT selectControl(
      const DiscreteReferenceTrajectoryClosest<StateT, ControlT> &trajectory,
      const StateT &current_state, std::chrono::duration<double> dt) = 0;
};
