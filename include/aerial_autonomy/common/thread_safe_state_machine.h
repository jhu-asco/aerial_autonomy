#pragma once

#include <boost/msm/back/state_machine.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace boost {
namespace msm {
namespace back {
template <class A0, class A1 = parameter::void_, class A2 = parameter::void_,
          class A3 = parameter::void_, class A4 = parameter::void_>
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

  // Main function used by clients of the derived FSM to make transitions.
  template <class Event> execute_return process_event(Event const &evt) {
    recursive_mutex::scoped_lock lock(process_event_mutex_);
    return this->process_event_internal(evt, true);
  }
};
}
}
} // boost::msm::back
