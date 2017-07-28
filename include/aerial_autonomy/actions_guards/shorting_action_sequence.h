#pragma once

#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/exec_if.h>
#include <boost/mpl/apply.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/begin_end.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/mpl/deref.hpp>
#include <boost/mpl/is_sequence.hpp>
#include <boost/mpl/next_prior.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/type_traits/is_same.hpp>

namespace boost {
namespace msm {
namespace front {
template <class Sequence> struct ShortingActionSequence_ {
  typedef Sequence sequence;
  // if one functor of the sequence defers events, the complete sequence does
  typedef ::boost::mpl::bool_<
      ::boost::mpl::count_if<
          sequence,
          has_deferring_action<::boost::mpl::placeholders::_1>>::value != 0>
      some_deferring_actions;

  template <class Event, class FSM, class STATE> struct state_action_result {
    typedef void type;
  };
  template <class EVT, class FSM, class STATE> struct Call {
    Call(EVT const &evt, FSM &fsm, STATE &state)
        : evt_(evt), fsm_(fsm), state_(state) {}
    template <class FCT> bool operator()(::boost::msm::wrap<FCT> const &) {
      return FCT()(evt_, fsm_, state_);
    }

  private:
    EVT const &evt_;
    FSM &fsm_;
    STATE &state_;
  };
  template <class EVT, class FSM, class SourceState, class TargetState>
  struct transition_action_result {
    typedef void type;
  };
  template <class EVT, class FSM, class SourceState, class TargetState>
  struct Call2 {
    Call2(EVT const &evt, FSM &fsm, SourceState &src, TargetState &tgt)
        : evt_(evt), fsm_(fsm), src_(src), tgt_(tgt) {}
    template <class FCT> bool operator()(::boost::msm::wrap<FCT> const &) {
      return FCT()(evt_, fsm_, src_, tgt_);
    }

  private:
    EVT const &evt_;
    FSM &fsm_;
    SourceState &src_;
    TargetState &tgt_;
  };

  typedef ::boost::mpl::set<state_action_tag, action_tag> tag_type;

  template <class EVT, class FSM, class STATE>
  bool operator()(EVT const &evt, FSM &fsm, STATE &state) {
    return ::boost::mpl::exec_if<
        Sequence, boost::msm::wrap<::boost::mpl::placeholders::_1>>(
        Call<EVT, FSM, STATE>(evt, fsm, state));
  }
  template <class EVT, class FSM, class SourceState, class TargetState>
  bool operator()(EVT const &evt, FSM &fsm, SourceState &src,
                  TargetState &tgt) {
    return ::boost::mpl::exec_if<
        Sequence, boost::msm::wrap<::boost::mpl::placeholders::_1>>(
        Call2<EVT, FSM, SourceState, TargetState>(evt, fsm, src, tgt));
  }
};
}
}
}
