#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/arm_events.h>
#include <aerial_autonomy/robot_systems/arm_system.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <arm_parsers/arm_simulator.h>
#include <gtest/gtest.h>
#include <typeindex>

/**
 * @brief Sample Logic state machine for testing arm system
 */
using ArmLogicStateMachine = SampleLogicStateMachine_<ArmSystem>;

/**
* @brief Namespace for pick, place and arm states and actions
*/
using psa = PickPlaceStatesActions<ArmLogicStateMachine>;

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace ae = arm_events;

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

// Status
using ArmStatusInternalActionFunctor =
    ArmStatusInternalActionFunctor_<ArmLogicStateMachine>;

// Folding
using ArmFoldInternalActionFunctor =
    ArmFoldInternalActionFunctor_<ArmLogicStateMachine>;

class ArmFunctorTests : public ::testing::Test {
protected:
  std::shared_ptr<ArmSimulator> arm_hardware;
  ArmSystem arm_system;
  ArmLogicStateMachine sample_logic_state_machine;
  int dummy_start_state, dummy_target_state;

public:
  ArmFunctorTests()
      : arm_hardware(new ArmSimulator),
        arm_system(std::dynamic_pointer_cast<ArmParser>(arm_hardware)),
        sample_logic_state_machine(arm_system), dummy_start_state(0),
        dummy_target_state(0) {}
};

/// \brief Test Arm Status Functor
TEST_F(ArmFunctorTests, Constructor) {
  ASSERT_NO_THROW(new ArmStatusInternalActionFunctor());
}

TEST_F(ArmFunctorTests, CallOperatorFunction) {
  arm_system.power(true);
  // Check arm status
  ArmStatusInternalActionFunctor arm_status_internal_action_functor;
  ASSERT_TRUE(arm_status_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state));
  arm_system.power(false);
  ASSERT_FALSE(arm_status_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state));
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}
///

/// \brief Test Folding
TEST_F(ArmFunctorTests, Folding) {
  ASSERT_NO_THROW(new ArmFoldInternalActionFunctor());
}

TEST_F(ArmFunctorTests, FoldingCall) {
  arm_system.power(true);
  // Try folding
  ArmFoldInternalActionFunctor arm_folding_internal_action_functor;
  ASSERT_TRUE(arm_folding_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state));
  // If arm is powered off should abort
  arm_system.power(false);
  ASSERT_FALSE(arm_folding_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state));
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
  // If arm if folded should process completed
  arm_system.power(true);
  arm_system.foldArm();
  ASSERT_FALSE(arm_folding_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state));
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}
///

/// \brief Test Transition actions
TEST_F(ArmFunctorTests, TransitionActionsFolding) {
  psa::ArmFold arm_fold_functor;
  arm_system.power(true);
  ASSERT_FALSE(arm_system.getCommandStatus());
  arm_fold_functor(ae::Fold(), sample_logic_state_machine, dummy_start_state,
                   dummy_target_state);
  ASSERT_TRUE(arm_system.getCommandStatus());
  // If powered off, cannot fold
  arm_system.power(false);
  arm_fold_functor(ae::Fold(), sample_logic_state_machine, dummy_start_state,
                   dummy_target_state);
  ASSERT_FALSE(arm_system.getCommandStatus());
}

TEST_F(ArmFunctorTests, TransitionActionsRightArm) {
  psa::ArmRightFold arm_right_fold_functor;
  arm_system.power(true);
  ASSERT_FALSE(arm_system.getCommandStatus());
  arm_right_fold_functor(ae::RightAngleFold(), sample_logic_state_machine,
                         dummy_start_state, dummy_target_state);
  ASSERT_TRUE(arm_system.getCommandStatus());
  // If powered off, cannot right fold
  arm_system.power(false);
  arm_right_fold_functor(ae::RightAngleFold(), sample_logic_state_machine,
                         dummy_start_state, dummy_target_state);
  ASSERT_FALSE(arm_system.getCommandStatus());
}

TEST_F(ArmFunctorTests, TransitionActionsGrip) {
  psa::ArmGripAction<true> arm_grip_functor;
  arm_system.power(true);
  ASSERT_FALSE(arm_system.getCommandStatus());
  arm_grip_functor(ae::Grip(), sample_logic_state_machine, dummy_start_state,
                   dummy_target_state);
  ASSERT_TRUE(arm_system.getCommandStatus());
  ASSERT_TRUE(arm_hardware->getGripperValue());
  // If powered off, cannot grip
  arm_system.power(false);
  arm_grip_functor(ae::Grip(), sample_logic_state_machine, dummy_start_state,
                   dummy_target_state);
  ASSERT_FALSE(arm_system.getCommandStatus());
}

TEST_F(ArmFunctorTests, TransitionActionsUnGrip) {
  psa::ArmGripAction<false> arm_ungrip_functor;
  arm_system.power(true);
  ASSERT_FALSE(arm_system.getCommandStatus());
  arm_ungrip_functor(ae::UnGrip(), sample_logic_state_machine,
                     dummy_start_state, dummy_target_state);
  ASSERT_TRUE(arm_system.getCommandStatus());
  ASSERT_FALSE(arm_hardware->getGripperValue());
  // If powered off, cannot grip
  arm_system.power(false);
  arm_ungrip_functor(ae::Grip(), sample_logic_state_machine, dummy_start_state,
                     dummy_target_state);
  ASSERT_FALSE(arm_system.getCommandStatus());
  ASSERT_FALSE(arm_hardware->getGripperValue());
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
