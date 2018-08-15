#pragma once
#include "aerial_autonomy/types/discrete_reference_trajectory.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

/**
* @brief A discrete trajectory containing controls, states and timestamps
* Gets state/control information using linear interpolation
*
* @tparam StateT The type of state stored along the trajectory
* @tparam ControlT The type of control along the trajectory
*/
template <class StateT, class ControlT>
struct DiscreteReferenceTrajectoryInterpolate
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
    int i = closest_t - this->ts.begin();

    if (i == 0) {
      return std::pair<StateT, ControlT>(this->states.at(0),
                                         this->controls.at(0));
    } else {
      if (std::fabs(this->ts.at(i) - this->ts.at(i - 1)) < 1e-7) {
        throw std::logic_error("Times are too close together");
      }
      double weight =
          (t - this->ts.at(i - 1)) / (this->ts.at(i) - this->ts.at(i - 1));
      return std::pair<StateT, ControlT>(
          this->states.at(i) * weight + this->states.at(i - 1) * (1 - weight),
          this->controls.at(i) * weight +
              this->controls.at(i - 1) * (1 - weight));
    }
  }
  /**
  * @brief Gets the trajectory information at the end
  *
  * @return Trajectory state and control
  */
  std::pair<StateT, ControlT> atGoalEnd() const {
    return std::pair<StateT, ControlT>(this->states.back(),
                                       this->controls.back());
  }
};
