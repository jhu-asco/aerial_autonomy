#pragma once

#include <boost/msm/back/state_machine.hpp>
#include <boost/thread/recursive_mutex.hpp>

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

  boost::recursive_mutex
      process_event_mutex_; ///< Mutex to synchronize process event functions

public:
  thread_safe_state_machine<A0, A1, A2, A3, A4>()
      : state_machine<A0, A1, A2, A3, A4>() {}

  template <class Expr>
  thread_safe_state_machine<A0, A1, A2, A3, A4>(Expr const &expr)
      : state_machine<A0, A1, A2, A3, A4>(expr) {}

  // all state machines are friend with each other to allow embedding any of
  // them in another fsm
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
    return this->process_event_internal(evt, true);
  }
};
}
}
} // boost::msm::back
