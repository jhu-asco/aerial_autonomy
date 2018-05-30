#pragma once

#include "aerial_autonomy/types/reference_trajectory.h"

/**
* @brief A reference trajectory that always returns the same state and control,
* no matter the time
*/
template <class StateT, class ControlT>
class Waypoint : public ReferenceTrajectory<StateT, ControlT> {
public:
  /**
  * @brief Constructor
  * @param goal_state The goal state of the waypoint
  * @param goal_control The goal control of the waypoint
  */
  Waypoint(StateT goal_state, ControlT goal_control)
      : goal_state_(goal_state), goal_control_(goal_control) {}
  /**
  * @brief Gets the trajectory information at the specified time
  * @param t Time
  * @return Trajectory state and control
  */
  virtual std::pair<StateT, ControlT> atTime(double t) const {
    return std::pair<StateT, ControlT>(goal_state_, goal_control_);
  }

protected:
  /**
  * @brief Goal state
  */
  StateT goal_state_;
  /**
  * @brief Goal control
  */
  ControlT goal_control_;
};
