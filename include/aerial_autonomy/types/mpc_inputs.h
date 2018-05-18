#pragma once
#include <vector>

/**
* @brief Inputs to MPC optimization
*
* @tparam StateT type of initial state
*/
template <class StateT> struct MPCInputs {
  /**
  * @brief Initial condition for optimization
  */
  StateT initial_state;
  /**
  * @brief Dynamic constraints for optimization
  */
  std::vector<Constraint> constraints;
};
