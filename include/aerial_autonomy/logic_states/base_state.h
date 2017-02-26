#pragma once
/**
 * Base state for all states in logic state machine
 *
 * This class provides a wrapper for any action function and robot
 * system
 * Entry and exit function can also be overwritten by subclassing this base
 * state.
 */

// Boost Includes
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

// Include Base state:
#include <aerial_autonomy/actions_guards/base_functors.h>

// Static asserts
#include <type_traits>

namespace msmf = boost::msm::front;

template <class RobotSystemT, class LogicStateMachineT, class ActionFctr>
class BaseState : public msmf::state<> {
  // Perform static asserts to ensure the ActionFctr is valid:
  static_assert(
      std::is_base_of<InternalActionFunctor<RobotSystemT, LogicStateMachineT>,
                      ActionFctr>::value,
      "ActionFctr not a subclass of InternalActionFctr");

public:
  /**
   * @brief The internal_transition_table to call run function in every state
   */
  struct internal_transition_table
      : boost::mpl::vector<
            msmf::Internal<InternalTransitionEvent, ActionFctr, msmf::none>> {};

  /**
   * @brief Destructor
   */
  virtual ~BaseState() {}
};
