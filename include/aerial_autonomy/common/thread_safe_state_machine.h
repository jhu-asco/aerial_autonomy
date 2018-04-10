#pragma once

#include <boost/msm/back/state_machine.hpp>
#include <boost/thread/recursive_mutex.hpp>
// Type index
#include <typeindex>
// Internal transition event
#include <aerial_autonomy/types/internal_transition_event.h>

/**
* @brief Boost namespace
*/
namespace boost {
/**
* @brief Meta state machine namespace
*/
namespace msm {
/**
* @brief Backend namespace
*/
namespace back {
template <class A0, class A1 = parameter::void_, class A2 = parameter::void_,
          class A3 = parameter::void_, class A4 = parameter::void_>
/**
* @brief Thread safe state machine class that extends
* boost::msm::back::state_machine class
*/
class thread_safe_state_machine : public state_machine<A0, A1, A2, A3, A4> {

  /**
   * @brief Mutex to synchronize process event functions
   */
  mutable boost::recursive_mutex process_event_mutex_;
  /**
   * @brief  Last event processed by the state machine
   */
  std::type_index last_processed_event_index;

public:
  thread_safe_state_machine<A0, A1, A2, A3, A4>()
      : state_machine<A0, A1, A2, A3, A4>(),
        last_processed_event_index(typeid(NULL)) {}

  template <class Expr>
  thread_safe_state_machine<A0, A1, A2, A3, A4>(
      Expr const &expr,
      typename ::boost::enable_if<
          typename ::boost::proto::is_expr<Expr>::type>::type * = 0)
      : state_machine<A0, A1, A2, A3, A4>(expr),
        last_processed_event_index(typeid(NULL)) {}

/**
 * @brief Function that adds multiple arguments to backend state
 * machine
 *
 * @param z Recursive argument
 * @param n The number of constructor arguments
 * @param  unused argument
 *
 * @return Constructor with multiple arguments
 */
#define MSM_CONSTRUCTOR_HELPER_EXECUTE_SUB(z, n, unused) ARG##n t##n
#define MSM_CONSTRUCTOR_HELPER_EXECUTE(z, n, unused)                           \
  template <BOOST_PP_ENUM_PARAMS(n, class ARG)>                                \
  thread_safe_state_machine<A0, A1, A2, A3, A4>(                               \
      BOOST_PP_ENUM(n, MSM_CONSTRUCTOR_HELPER_EXECUTE_SUB, ~),                 \
      typename ::boost::disable_if<                                            \
          typename ::boost::proto::is_expr<ARG0>::type>::type *                \
      = 0)                                                                     \
      : state_machine<A0, A1, A2, A3, A4>(BOOST_PP_ENUM_PARAMS(n, t)),         \
        last_processed_event_index(typeid(NULL)) {}                            \
  template <class Expr, BOOST_PP_ENUM_PARAMS(n, class ARG)>                    \
  thread_safe_state_machine<A0, A1, A2, A3, A4>(                               \
      Expr const &expr,                                                        \
      BOOST_PP_ENUM(n, MSM_CONSTRUCTOR_HELPER_EXECUTE_SUB, ~),                 \
      typename ::boost::enable_if<                                             \
          typename ::boost::proto::is_expr<Expr>::type>::type * = 0)           \
      : state_machine<A0, A1, A2, A3, A4>(expr, BOOST_PP_ENUM_PARAMS(n, t)) {}

  /**
   * @brief Macro expands to a sequence to specified CONTRUCTOR ARG SIZE
   */
  BOOST_PP_REPEAT_FROM_TO(1, BOOST_PP_ADD(BOOST_MSM_CONSTRUCTOR_ARG_SIZE, 1),
                          MSM_CONSTRUCTOR_HELPER_EXECUTE, ~)
#undef MSM_CONSTRUCTOR_HELPER_EXECUTE
#undef MSM_CONSTRUCTOR_HELPER_EXECUTE_SUB

  // all state machines are friend with each other to allow embedding any of
  // them in another fsm
  /**
   * @brief Create friend for backend state machine to use private members
   *
   * @tparam class template parameters for the parent class
   * @tparam class template parameters for the parent class
   * @tparam class template parameters for the parent class
   * @tparam class template parameters for the parent class
   * @tparam class template parameters for the parent class
   */
  template <class, class, class, class, class>
  friend class boost::msm::back::state_machine;

  template <class, class, class, class, class>
  friend class boost::msm::back::thread_safe_state_machine;

  /**
  * @brief The process event function that triggers transition actions in state
  * machine
  * The function is thread safe but does not execute the events in any
  * particular order
  *
  * @tparam Event The event type that is received
  * @param evt Instance of event type received by this function
  *
  * @return Enum specifying whether the event is handled, deferred etc.
  */
  template <class Event> execute_return process_event(Event const &evt) {
    recursive_mutex::scoped_lock lock(process_event_mutex_);
    // Store the event if it is not internal transition event
    std::type_index event_index = typeid(Event);
    if (event_index != typeid(InternalTransitionEvent))
      last_processed_event_index = event_index;
    return this->process_event_internal(evt, true);
  }

  /**
   * @brief Returns the type index of last processed event after locking
   *
   * @return type index of last processed event
   */
  std::type_index lastProcessedEventIndex() const {
    recursive_mutex::scoped_lock lock(process_event_mutex_);
    return last_processed_event_index;
  }
};
}
}
} // boost::msm::back
