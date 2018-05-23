#pragma once
#include "aerial_autonomy/types/discrete_reference_trajectory.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

/**
* @brief A discrete trajectory containing controls, states and timestamps
* Gets state/control information using the closest time to the specified time
*
* @tparam StateT The type of state stored along the trajectory
* @tparam ControlT The type of control along the trajectory
*/
template <class StateT, class ControlT>
struct DiscreteReferenceTrajectoryClosest
    : public DiscreteReferenceTrajectory<StateT, ControlT> {
public:
  /**
  * @brief Gets the trajectory information at the specified time using linear
  * interpolation
  * @param t Time
  * @return Trajectory state and control
  */
  std::pair<StateT, ControlT> atTime(double t) const {
    if (this->ts.empty() || t < this->ts.front() || t > this->ts.back()) {
      throw std::out_of_range("Accessed reference trajectory out of bounds");
    }
    auto closest_t = std::lower_bound(this->ts.begin(), this->ts.end(), t);
    int closest_i = closest_t - this->ts.begin();
    if (closest_i > 0) {
      if (std::fabs(*closest_t - t) >
          std::fabs(this->ts.at(closest_i - 1) - t)) {
        closest_i = closest_i - 1;
      }
    }

    return std::pair<StateT, ControlT>(this->states.at(closest_i),
                                       this->controls.at(closest_i));
  }
};
