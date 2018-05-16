#pragma once

/**
* @brief Estimate the state of the robot
*
* @tparam StateT The state type estimated by the estimator
* @tparam ControlT  The control type used to propagate the estimator
*/
template <class StateT, class ControlT> struct AbstractStateEstimator {
  /**
  * @brief Run estimator loop. Propagates the estimator state using controls
  *
  * @param control Current control
  */
  virtual void propagate(const ControlT &control) = 0;
  /**
  * @brief Get the current estimate of state
  *
  * @return The initial state estimate
  */
  virtual StateT getState() = 0;
  /**
   * @brief check if estimator is fine
   * @return true if estimator is ok
   */
  virtual bool getStatus() { return true; }
};
