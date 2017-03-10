#include <gtest/gtest.h>

// thread safe back-end
#include <aerial_autonomy/common/thread_safe_state_machine.h>

// front-end
#include <boost/msm/front/state_machine_def.hpp>

// boost thread
#include <boost/thread.hpp>

// functor row
#include <boost/msm/front/functor_row.hpp>

namespace msmf = boost::msm::front;

// Event:
struct Count {};
struct DoubleCountEvt {};
struct SwitchToDoubleCount {};

// Forward Declaration
struct SampleStateMachineFrontEnd;
using SampleStateMachine =
    boost::msm::back::thread_safe_state_machine<SampleStateMachineFrontEnd>;

// Create a state machine front end to count a number when called from multiple
// threads
struct SampleStateMachineFrontEnd
    : public msmf::state_machine_def<SampleStateMachineFrontEnd> {
  int count_value;

  SampleStateMachineFrontEnd(int initial_count_value)
      : count_value(initial_count_value) {}

  struct SingleCount : msmf::state<> {};
  struct DoubleCount : msmf::state<> {};

  /**
  * @brief Initial state for state machine
  */
  using initial_state = SingleCount;

  /**
  * @brief Count action functor
  */
  struct count_action_fct {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &, FSM &fsm, SourceState &, TargetState &) {
      static_assert(
          std::is_base_of<FSM, SampleStateMachine>::value,
          "Template Logic state machine arg is not subclass of provided FSM");
      ++fsm.count_value;
    }
  };

  /**
  * @brief Double count action functor
  */
  struct double_count_action_fct {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &, FSM &fsm, SourceState &, TargetState &) {
      static_assert(
          std::is_base_of<FSM, SampleStateMachine>::value,
          "Template Logic state machine arg is not subclass of provided FSM");
      SampleStateMachine *fsm_cast = static_cast<SampleStateMachine *>(&fsm);
      fsm_cast->process_event(Count()); // Count twice
      fsm_cast->process_event(Count());
    }
  };
  /**
  * @brief Make transition table cleaner
  */
  using p = SampleStateMachineFrontEnd;

  struct transition_table
      : boost::mpl::vector<msmf::Row<SingleCount, Count, msmf::none,
                                     count_action_fct, msmf::none>,
                           msmf::Row<SingleCount, SwitchToDoubleCount,
                                     DoubleCount, msmf::none, msmf::none>,
                           msmf::Row<DoubleCount, Count, msmf::none,
                                     count_action_fct, msmf::none>,
                           msmf::Row<DoubleCount, DoubleCountEvt, msmf::none,
                                     double_count_action_fct, msmf::none>> {};
};

void countTillHundred(SampleStateMachine *state_machine) {
  for (int i = 0; i < 100; ++i) {
    state_machine->process_event(Count());
  }
}

void doubleCountTillHundred(SampleStateMachine *state_machine) {
  for (int i = 0; i < 100; ++i) {
    state_machine->process_event(DoubleCountEvt());
  }
}

TEST(ThreadSafeStateMachineTests, Constructor) {
  ASSERT_NO_THROW(new SampleStateMachine(0));
}

TEST(ThreadSafeStateMachineTests, SingleCount) {
  SampleStateMachine state_machine(1);
  ASSERT_NO_THROW(state_machine.start());
  ASSERT_NO_THROW(state_machine.process_event(Count()));
  ASSERT_EQ(state_machine.count_value, 2);
}

TEST(ThreadSafeStateMachineTests, DoubleCount) {
  SampleStateMachine state_machine(0);
  ASSERT_NO_THROW(state_machine.start());
  ASSERT_NO_THROW(state_machine.process_event(SwitchToDoubleCount()));
  ASSERT_NO_THROW(state_machine.process_event(DoubleCountEvt()));
  ASSERT_EQ(state_machine.count_value, 2);
}

TEST(ThreadSafeStateMachineTests, MultiThreadCount) {
  SampleStateMachine state_machine(0);
  state_machine.start();
  boost::thread t1(countTillHundred, &state_machine);
  boost::thread t2(countTillHundred, &state_machine);
  t1.join();
  t2.join();
  ASSERT_EQ(state_machine.count_value, 200);
}

TEST(ThreadSafeStateMachineTests, MultiThreadDoubleCount) {
  SampleStateMachine state_machine(0);
  state_machine.start();
  state_machine.process_event(SwitchToDoubleCount());
  boost::thread t1(countTillHundred, &state_machine);
  boost::thread t2(doubleCountTillHundred, &state_machine);
  t1.join();
  t2.join();
  ASSERT_EQ(state_machine.count_value, 300);
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
