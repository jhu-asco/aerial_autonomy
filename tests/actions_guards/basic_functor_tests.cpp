#include <gtest/gtest.h>
#include <aerial_autonomy/tests/sample_parser.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/actions_guards/basic_states.h>
#include <typeindex>
// Land
using LandTransitionActionFunctor =
    LandTransitionActionFunctor_<UAVLogicStateMachine>;
using LandInternalActionFunctor =
    LandInternalActionFunctor_<UAVLogicStateMachine>;
// Hovering
using HoveringInternalActionFunctor =
    HoveringInternalActionFunctor_<UAVLogicStateMachine>;
// Takeoff
using TakeoffTransitionActionFunctor =
    TakeoffTransitionActionFunctor_<UAVLogicStateMachine>;
using TakeoffTransitionGuardFunctor =
    TakeoffTransitionGuardFunctor_<UAVLogicStateMachine>;
using TakeoffAbortActionFunctor =
    TakeoffAbortActionFunctor_<UAVLogicStateMachine>;
using TakeoffInternalActionFunctor =
    TakeoffInternalActionFunctor_<UAVLogicStateMachine>;
// Takeoff
using PositionControlTransitionActionFunctor =
    PositionControlTransitionActionFunctor_<UAVLogicStateMachine>;
using PositionControlTransitionGuardFunctor =
    PositionControlTransitionGuardFunctor_<UAVLogicStateMachine>;
using PositionControlAbortActionFunctor =
    PositionControlAbortActionFunctor_<UAVLogicStateMachine>;
using PositionControlInternalActionFunctor =
    PositionControlInternalActionFunctor_<UAVLogicStateMachine>;

/// \brief Test LandFunctor
TEST(LandFunctorTests, Constructor) {
  ASSERT_NO_THROW(new LandTransitionActionFunctor());
  ASSERT_NO_THROW(new LandInternalActionFunctor());
}

TEST(LandFunctorTests, CallOperatorFunction) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  LandTransitionActionFunctor land_transition_action_functor;
  int dummy_start_state, dummy_target_state;
  land_transition_action_functor(Land(), sample_logic_state_machine,
                                 dummy_start_state, dummy_target_state);
  ASSERT_STREQ(uav_system.getUAVData().quadstate.c_str(), "land");
  // Internal Action
  LandInternalActionFunctor land_internal_action_functor;
  drone_hardware.setaltitude(2.0);
  land_internal_action_functor(InternalTransitionEvent(),
                               sample_logic_state_machine, dummy_start_state,
                               dummy_target_state);
  ASSERT_NE(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
  // After setting correct altitude
  drone_hardware.setaltitude(0.0);
  land_internal_action_functor(InternalTransitionEvent(),
                               sample_logic_state_machine, dummy_start_state,
                               dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}
///

/// \brief Test HoveringFunctor
TEST(HoveringFunctorTests, Constructor) {
  ASSERT_NO_THROW(new HoveringInternalActionFunctor());
}

TEST(HoveringFunctorTests, CallOperatorFunction) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  // Internal Action
  HoveringInternalActionFunctor hovering_internal_action_functor;
  drone_hardware.setBatteryPercent(60);
  hovering_internal_action_functor(InternalTransitionEvent(),
                                   sample_logic_state_machine,
                                   dummy_start_state, dummy_target_state);
  ASSERT_NE(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Land)));
  // After setting correct altitude
  drone_hardware.setBatteryPercent(20);
  hovering_internal_action_functor(InternalTransitionEvent(),
                                   sample_logic_state_machine,
                                   dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Land)));
}
///

/// \brief Test Takeoff Functors
TEST(TakeoffFunctorTests, Constructor) {
  ASSERT_NO_THROW(new TakeoffInternalActionFunctor());
  ASSERT_NO_THROW(new TakeoffTransitionActionFunctor());
  ASSERT_NO_THROW(new TakeoffTransitionGuardFunctor());
  ASSERT_NO_THROW(new TakeoffAbortActionFunctor());
}

TEST(TakeoffFunctorTests, TransitionActionTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  TakeoffTransitionActionFunctor takeoff_transition_action_functor;
  takeoff_transition_action_functor(Takeoff(), sample_logic_state_machine,
                                    dummy_start_state, dummy_target_state);
  ASSERT_STREQ(uav_system.getUAVData().quadstate.c_str(), "takeoff");
}

TEST(TakeoffFunctorTests, AbortActionTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  TakeoffAbortActionFunctor takeoff_abort_action_functor;
  takeoff_abort_action_functor(Abort(), sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  ASSERT_STREQ(uav_system.getUAVData().quadstate.c_str(), "land");
}

TEST(TakeoffFunctorTests, TransitionGuardTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  TakeoffTransitionGuardFunctor takeoff_transition_guard_functor;
  drone_hardware.setBatteryPercent(60);
  bool result =
      takeoff_transition_guard_functor(Takeoff(), sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state);
  ASSERT_TRUE(result);
  drone_hardware.setBatteryPercent(10);
  result =
      takeoff_transition_guard_functor(Takeoff(), sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state);
  ASSERT_FALSE(result);
}

TEST(TakeoffFunctorTests, InternalActionTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  TakeoffInternalActionFunctor takeoff_internal_action_functor;
  drone_hardware.setaltitude(0.0);
  takeoff_internal_action_functor(InternalTransitionEvent(),
                                  sample_logic_state_machine, dummy_start_state,
                                  dummy_target_state);
  ASSERT_NE(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
  // After setting correct altitude
  drone_hardware.setaltitude(2.0);
  takeoff_internal_action_functor(InternalTransitionEvent(),
                                  sample_logic_state_machine, dummy_start_state,
                                  dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}
///

/// \brief Test Takeoff Functors
TEST(PositionControlFunctorTests, Constructor) {
  ASSERT_NO_THROW(new PositionControlInternalActionFunctor());
  ASSERT_NO_THROW(new PositionControlTransitionActionFunctor());
  ASSERT_NO_THROW(new PositionControlTransitionGuardFunctor());
  ASSERT_NO_THROW(new PositionControlAbortActionFunctor());
}

TEST(PositionControlFunctorTests, TransitionActionTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  PositionControlTransitionActionFunctor
      position_control_transition_action_functor;
  PositionYaw goal(1, 1, 1, 1);
  position_control_transition_action_functor(
      goal, sample_logic_state_machine, dummy_start_state, dummy_target_state);
  PositionYaw resulting_goal =
      uav_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
  ASSERT_EQ(goal, resulting_goal);
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  PositionYaw data_position_yaw(data.localpos.x, data.localpos.y,
                                data.localpos.z, data.rpydata.z);
  ASSERT_EQ(data_position_yaw, goal);
}

TEST(PositionControlFunctorTests, AbortActionTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  PositionControlAbortActionFunctor position_control_abort_action_functor;
  PositionYaw goal(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(goal);
  position_control_abort_action_functor(Abort(), sample_logic_state_machine,
                                        dummy_start_state, dummy_target_state);
  // Since the controller is aborted, will not run the controller
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  PositionYaw data_position_yaw(data.localpos.x, data.localpos.y,
                                data.localpos.z, data.rpydata.z);
  ASSERT_NE(data_position_yaw, goal);
}

TEST(PositionControlFunctorTests, TransitionGuardTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  PositionYaw goal(1, 1, 1, 1);
  PositionControlTransitionGuardFunctor
      position_control_transition_guard_functor;
  bool result = position_control_transition_guard_functor(
      goal, sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_TRUE(result);
  goal = PositionYaw(1, 1, 1000, 1);
  result = position_control_transition_guard_functor(
      goal, sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_FALSE(result);
}

TEST(PositionControlFunctorTests, InternalActionTest) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  PositionControlInternalActionFunctor position_control_internal_action_functor;
  PositionYaw goal(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(goal);
  position_control_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state);
  ASSERT_NE(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
  // After running the active controller once
  uav_system.runActiveController(HardwareType::UAV);
  position_control_internal_action_functor(
      InternalTransitionEvent(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
