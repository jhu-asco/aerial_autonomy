#pragma once
#include <algorithm>
#include <stdexcept>
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

  /**
  * @brief Gets the trajectory information at the specified time using linear
  * interpolation
  * @param t Time
  * @return Trajectory state and control
  */
  std::pair<StateT, ControlT> atTime(double t) const {
    if (ts.empty() || t < ts.front() || t > ts.back()) {
      throw std::out_of_range("Accessed reference trajectory out of bounds");
    }
    auto closest_t = std::lower_bound(ts.begin(), ts.end(), t);
    int i = closest_t - ts.begin();

    if (*closest_t == t) {
      return std::pair<StateT, ControlT>(states.at(i), controls.at(i));
    } else {
      double weight = (t - ts.at(i - 1)) / (ts.at(i) - ts.at(i - 1));
      return std::pair<StateT, ControlT>(
          states.at(i) * weight + states.at(i - 1) * (1 - weight),
          controls.at(i) * weight + controls.at(i - 1) * (1 - weight));
    }
  }
};
