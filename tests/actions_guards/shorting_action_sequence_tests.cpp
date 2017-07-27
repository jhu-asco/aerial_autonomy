#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <gtest/gtest.h>
#include <typeindex>

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

/**
 * Dummy Internal action
 *
 * @tparam return_value This is returned by the action functor
 */
template <bool return_value>
struct TestInternalActionFunctor
    : InternalActionFunctor<EmptyRobotSystem, SampleLogicStateMachine> {
  bool action(EmptyRobotSystem &, SampleLogicStateMachine &) {
    return return_value;
  }
};

/**
 * @brief Internal action that processes takeoff event using state machine
 *
 */
struct TakeoffInternalActionFunctor
    : InternalActionFunctor<EmptyRobotSystem, SampleLogicStateMachine> {
  bool action(EmptyRobotSystem &, SampleLogicStateMachine &fsm) {
    fsm.process_event(be::Takeoff());
    return true;
  }
};

/**
 * @brief Internal action that processes land event using state machine
 *
 */
struct LandInternalActionFunctor
    : InternalActionFunctor<EmptyRobotSystem, SampleLogicStateMachine> {
  bool action(EmptyRobotSystem &, SampleLogicStateMachine &fsm) {
    fsm.process_event(be::Land());
    return true;
  }
};

/// \brief Test Creating TestInternalActionFunctor
TEST(ShortingActionSequenceTests, CreateInternalActionFunctor) {
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine sample_logic_state_machine(robot_system);
  TestInternalActionFunctor<true> internal_action_functor_true;
  EXPECT_TRUE(internal_action_functor_true.action(robot_system,
                                                  sample_logic_state_machine));
  TestInternalActionFunctor<false> internal_action_functor_false;
  EXPECT_FALSE(internal_action_functor_false.action(
      robot_system, sample_logic_state_machine));
}
///
/// \brief Test Shorting Action Sequence
TEST(ShortingActionSequenceTests, CallShortingActionSequence) {
  typedef TestInternalActionFunctor<false> TestActionFunctorFalse;
  typedef TestInternalActionFunctor<true> TestActionFunctorTrue;
  boost::msm::front::ShortingActionSequence_<
      boost::mpl::vector<TestActionFunctorTrue, TestActionFunctorFalse>>
      shorting_action_sequence;
  int dummy_event, dummy_source_state, dummy_target_state;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine sample_logic_state_machine(robot_system);
  EXPECT_TRUE(shorting_action_sequence(dummy_event, sample_logic_state_machine,
                                       dummy_source_state, dummy_target_state));

  boost::msm::front::ShortingActionSequence_<
      boost::mpl::vector<TestActionFunctorFalse, TestActionFunctorFalse>>
      shorting_action_sequence_2;
  EXPECT_FALSE(
      shorting_action_sequence_2(dummy_event, sample_logic_state_machine,
                                 dummy_source_state, dummy_target_state));
}

TEST(ShortingActionSequenceTests, ShortingActionSequenceMidWay) {
  typedef TestInternalActionFunctor<false> TestActionFunctorFalse;
  typedef TestInternalActionFunctor<true> TestActionFunctorTrue;
  boost::msm::front::ShortingActionSequence_<
      boost::mpl::vector<TestActionFunctorFalse, TestActionFunctorTrue,
                         TestActionFunctorFalse, TakeoffInternalActionFunctor>>
      shorting_action_sequence;
  boost::msm::front::ShortingActionSequence_<
      boost::mpl::vector<TestActionFunctorFalse, TestActionFunctorFalse,
                         TakeoffInternalActionFunctor, TestActionFunctorTrue>>
      shorting_action_sequence_2;
  boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
      LandInternalActionFunctor, TakeoffInternalActionFunctor>>
      shorting_action_sequence_3;
  int dummy_event, dummy_source_state, dummy_target_state;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine sample_logic_state_machine(robot_system);
  EXPECT_TRUE(shorting_action_sequence(dummy_event, sample_logic_state_machine,
                                       dummy_source_state, dummy_target_state));
  EXPECT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(NULL)));
  EXPECT_TRUE(
      shorting_action_sequence_2(dummy_event, sample_logic_state_machine,
                                 dummy_source_state, dummy_target_state));
  EXPECT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(be::Takeoff)));
  EXPECT_TRUE(
      shorting_action_sequence_3(dummy_event, sample_logic_state_machine,
                                 dummy_source_state, dummy_target_state));
  EXPECT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(be::Land)));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
