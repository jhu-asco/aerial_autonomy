#pragma once
#include "aerial_autonomy/types/reference_trajectory.h"

/**
* @brief Select the control to be applied given the reference trajectory and
* current system state
*
* @tparam StateT The type of state
* @tparam ControlT The type of control
*/
template <class StateT, class ControlT> struct AbstractControlSelector {
  /**
  * @brief Select the control given reference trajectory and current state
  *
  * @param trajectory input reference trajectory
  * @param current_state Current system state
  *
  * @return the control to be applied
  */
  virtual ControlT
  selectControl(const ReferenceTrajectory<StateT, ControlT> &trajectory,
                const StateT &current_state) = 0;
};
