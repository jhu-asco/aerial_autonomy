#pragma once
/**
 * Base state for all states in logic state machine
 *
 * This class provides a wrapper for any action and guard function and robot
 * system
 * Entry and exit function can also be overwritten by subclassing this base
 * state.
 */

// Boost Includes
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

// Include Base state:
#include <aerial_autonomy/logic_states/base_functors.h>

// Static asserts
#include <type_traits>

namespace msmf = boost::msm::front;

template <class RobotSystemT, class LogicStateMachineT, class ActionFctr,
          class GuardFctr = msmf::none>
class BaseState : msmf::state<> {
  // Perform static asserts to ensure the ActionFctr and GuardFctr are valid:
  static_assert(
      (std::is_base_of<BaseRunFunctor<RobotSystemT, LogicStateMachineT>, ActionFctr>::value ||
       std::is_same<ActionFctr, msmf::none>::value),
      "ActionFctr not a subclass of BaseRunFctr");
  static_assert((std::is_base_of<BaseGuardFunctor<RobotSystemT, LogicStateMachineT>,
                                 GuardFctr>::value ||
                 std::is_same<GuardFctr, msmf::none>::value),
                "ActionFctr not a subclass of BaseRunFctr");
  /**
   * @brief The internal_transition_table to call run function in every state
   */
  struct internal_transition_table
      : boost::mpl::vector<
            msmf::Internal<RobotSystemT, ActionFctr, GuardFctr>> {};

public:
  /**
   * @brief Destructor
   */
  virtual ~BaseState() {}
};
