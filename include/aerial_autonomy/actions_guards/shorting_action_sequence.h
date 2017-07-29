#pragma once

/**
 * Modified action sequence that executes actions
 * until one of the actions returns true. Unlike
 * regular executions, this assumes the actions
 * return true false based on whether they processed
 * events on state machine. This is useful to avoid
 * running further actions when state transitions
 * occured in the current action
 *
 * Modified by Gowtham Garimella.
 * email ggarime1[at]jhu.edu
 */

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

/**
 * @brief namespace for boost library
 */
namespace boost {
/**
 * @brief namespace for state machine module
 */
namespace msm {
/**
 * @brief front end of state machine module
 */
namespace front {
/**
 * @brief Action sequence that runs until one of the actions returns false
 *
 * @tparam Sequence The Sequence of elements that are instantiated and evaluated
 */
template <class Sequence> struct ShortingActionSequence_ {
  /**
   * @brief Sequence of elements that are instantiated and evaluated
   */
  typedef Sequence sequence;
  // if one functor of the sequence defers events, the complete sequence does
  /**
   * @brief Deferring actions for the sequence
   */
  typedef ::boost::mpl::bool_<
      ::boost::mpl::count_if<
          sequence,
          has_deferring_action<::boost::mpl::placeholders::_1>>::value != 0>
      some_deferring_actions;

  /**
   * @brief Copied from boost main file. Not sure what this is for!
   *
   * @tparam Event Event type in state machine
   * @tparam FSM  State machine used to evaluate sequence
   * @tparam STATE State we are currently in
   */
  template <class Event, class FSM, class STATE> struct state_action_result {
    /**
     * @brief Define the result type to void maybe?
     */
    typedef void type;
  };
  /**
   * @brief Functor used to instantiate and evaluate elements in the sequence
   *
   * @tparam EVT Event type
   * @tparam FSM State machine type
   * @tparam STATE State type
   */
  template <class EVT, class FSM, class STATE> struct Call {
    /**
     * @brief Constructor to store event, state machine, state
     *
     * @param evt Event instance
     * @param fsm State machine
     * @param state State instance
     */
    Call(EVT const &evt, FSM &fsm, STATE &state)
        : evt_(evt), fsm_(fsm), state_(state) {}
    /**
     * @brief operator function that instantiates the functor and evaluates it
     * on the event, fsm, state
     *
     * @tparam FCT Functor type
     * @param  functor instance
     *
     * @return result of evaluating the functor
     */
    template <class FCT> bool operator()(::boost::msm::wrap<FCT> const &) {
      return FCT()(evt_, fsm_, state_);
    }

  private:
    EVT const &evt_; ///< Event to call with functor
    FSM &fsm_;       ///< State machine to use with functor
    STATE &state_;   ///< State to use with functor
  };
  /**
   * @brief Not sure what this is! Copied from boost main file
   *
   * @tparam EVT Event type
   * @tparam FSM State machine type
   * @tparam SourceState Source state type
   * @tparam TargetState Target state type
   */
  template <class EVT, class FSM, class SourceState, class TargetState>
  struct transition_action_result {
    /**
     * @brief Type of action result
     */
    typedef void type;
  };
  /**
   * @brief Functor used to instantiate and evaluate elements in the sequence
   *
   * @tparam EVT Event type
   * @tparam FSM State machine type
   * @tparam SourceState source state of action in state machine
   * @tparam TargetState target state of action in state machine
   */
  template <class EVT, class FSM, class SourceState, class TargetState>
  struct Call2 {
    /**
     * @brief Constructor to store event, statemachine, source, target
     *
     * @param evt event instance
     * @param fsm statemachine instance
     * @param src source state instance
     * @param tgt target state instance
     */
    Call2(EVT const &evt, FSM &fsm, SourceState &src, TargetState &tgt)
        : evt_(evt), fsm_(fsm), src_(src), tgt_(tgt) {}
    /**
     * @brief operator function that instantiates the functor and evaluates it
     * on the event, fsm, source, target state
     *
     * @tparam FCT Functor type
     * @param  functor instance
     *
     * @return result of evaluating the functor
     */
    template <class FCT> bool operator()(::boost::msm::wrap<FCT> const &) {
      return FCT()(evt_, fsm_, src_, tgt_);
    }

  private:
    EVT const &evt_;   ///< Event to call with functor
    FSM &fsm_;         ///< State machine to use with functor
    SourceState &src_; ///< Source state to use with functor
    TargetState &tgt_; ///< Target state to use with functor
  };

  /**
   * @brief Tag type
   */
  typedef ::boost::mpl::set<state_action_tag, action_tag> tag_type;

  /**
   * @brief operator to evaluate the action sequence
   *
   * @tparam EVT Event type
   * @tparam FSM Statemachine type
   * @tparam STATE  State type
   * @param evt event instance
   * @param fsm statemachine instance
   * @param state state instance
   *
   * @return  true if the action sequence returns true/false otherwise
   */
  template <class EVT, class FSM, class STATE>
  bool operator()(EVT const &evt, FSM &fsm, STATE &state) {
    return ::boost::mpl::exec_if<
        Sequence, boost::msm::wrap<::boost::mpl::placeholders::_1>>(
        Call<EVT, FSM, STATE>(evt, fsm, state));
  }
  /**
   * @brief operator to evaluate the action sequence
   *
   * @tparam EVT Event type
   * @tparam FSM Statemachine type
   * @tparam SourceState source state type
   * @tparam TargetState target state type
   * @param evt event instance
   * @param fsm statemachine instance
   * @param src source state instance
   * @param tgt target state instance
   *
   * @return  true if the action sequence returns true/false otherwise
   */
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
