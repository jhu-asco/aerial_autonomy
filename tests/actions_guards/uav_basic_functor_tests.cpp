#include <aerial_autonomy/actions_guards/uav_states_actions.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <typeindex>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief Namespace for basic states and actions for
* UAVLogicStateMachine
*/
using bsa = UAVStatesActions<UAVLogicStateMachine>;

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

// Land
using LandInternalActionFunctor =
    LandInternalActionFunctor_<UAVLogicStateMachine>;
// Hovering
using HoveringInternalActionFunctor =
    HoveringInternalActionFunctor_<UAVLogicStateMachine>;
// Takeoff
using TakeoffInternalActionFunctor =
    TakeoffInternalActionFunctor_<UAVLogicStateMachine>;
// Takeoff
using PositionControlInternalActionFunctor =
    PositionControlInternalActionFunctor_<UAVLogicStateMachine>;

/// \brief Test LandFunctor
TEST(LandFunctorTests, Constructor) {
  ASSERT_NO_THROW(new bsa::LandingAction());
  ASSERT_NO_THROW(new LandInternalActionFunctor());
}

TEST(LandFunctorTests, CallOperatorFunction) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  bsa::LandingAction land_transition_action_functor;
  int dummy_start_state, dummy_target_state;
  land_transition_action_functor(be::Land(), sample_logic_state_machine,
                                 dummy_start_state, dummy_target_state);
  ASSERT_STREQ(uav_system.getUAVData().quadstate.c_str(), "");
  // Internal Action
  LandInternalActionFunctor land_internal_action_functor;
  // Taking off which sets altitude to 0.5
  drone_hardware.takeoff();
  land_internal_action_functor(InternalTransitionEvent(),
                               sample_logic_state_machine, dummy_start_state,
                               dummy_target_state);
  ASSERT_NE(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
  // After landing which sets altitude to 0.0
  drone_hardware.land();
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
  QuadSimulator drone_hardware;
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
            std::type_index(typeid(be::Land)));
  // After setting correct altitude
  drone_hardware.setBatteryPercent(20);
  hovering_internal_action_functor(InternalTransitionEvent(),
                                   sample_logic_state_machine,
                                   dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(be::Land)));
}
///

/// \brief Test Takeoff Functors
TEST(TakeoffFunctorTests, Constructor) {
  ASSERT_NO_THROW(new TakeoffInternalActionFunctor());
  ASSERT_NO_THROW(new bsa::TakeoffAction());
  ASSERT_NO_THROW(new bsa::TakeoffGuard());
  ASSERT_NO_THROW(new bsa::TakeoffAbort());
}

TEST(TakeoffFunctorTests, TransitionActionTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  bsa::TakeoffAction takeoff_transition_action_functor;
  takeoff_transition_action_functor(be::Takeoff(), sample_logic_state_machine,
                                    dummy_start_state, dummy_target_state);
  ASSERT_STREQ(uav_system.getUAVData().quadstate.c_str(),
               "ARMED ENABLE_CONTROL ");
}

TEST(TakeoffFunctorTests, AbortActionTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  bsa::TakeoffAbort takeoff_abort_action_functor;
  takeoff_abort_action_functor(be::Abort(), sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  ASSERT_STREQ(uav_system.getUAVData().quadstate.c_str(), "");
}

TEST(TakeoffFunctorTests, TransitionGuardTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  bsa::TakeoffGuard takeoff_transition_guard_functor;
  drone_hardware.setBatteryPercent(60);
  bool result = takeoff_transition_guard_functor(
      be::Takeoff(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state);
  ASSERT_TRUE(result);
  drone_hardware.setBatteryPercent(10);
  result = takeoff_transition_guard_functor(
      be::Takeoff(), sample_logic_state_machine, dummy_start_state,
      dummy_target_state);
  ASSERT_FALSE(result);
}

TEST(TakeoffFunctorTests, InternalActionTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  TakeoffInternalActionFunctor takeoff_internal_action_functor;
  takeoff_internal_action_functor(InternalTransitionEvent(),
                                  sample_logic_state_machine, dummy_start_state,
                                  dummy_target_state);
  ASSERT_NE(sample_logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
  // After setting correct altitude
  drone_hardware.takeoff();
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
  ASSERT_NO_THROW(new bsa::ReachingGoalSet());
  ASSERT_NO_THROW(new bsa::ReachingGoalGuard());
  ASSERT_NO_THROW(new bsa::ReachingGoalAbort());
}

TEST(PositionControlFunctorTests, TransitionActionTest) {
  QuadSimulator drone_hardware;
  drone_hardware.takeoff();
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  bsa::ReachingGoalSet position_control_transition_action_functor;
  PositionYaw goal(1, 1, 1, 1);
  position_control_transition_action_functor(
      goal, sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_EQ(uav_system.getStatus<PositionControllerDroneConnector>(),
            ControllerStatus::Active);
  PositionYaw resulting_goal =
      uav_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
  ASSERT_EQ(goal, resulting_goal);
  uav_system.runActiveController(HardwareType::UAV);
  ASSERT_EQ(uav_system.getStatus<PositionControllerDroneConnector>(),
            ControllerStatus::Active);
  parsernode::common::quaddata data = uav_system.getUAVData();
  PositionYaw data_position_yaw(data.localpos.x, data.localpos.y,
                                data.localpos.z, data.rpydata.z);
  ASSERT_EQ(data_position_yaw, goal);
  uav_system.runActiveController(HardwareType::UAV);
  ASSERT_EQ(uav_system.getStatus<PositionControllerDroneConnector>(),
            ControllerStatus::Completed);
}

TEST(PositionControlFunctorTests, AbortActionTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  bsa::ReachingGoalAbort position_control_abort_action_functor;
  PositionYaw goal(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(goal);
  position_control_abort_action_functor(be::Abort(), sample_logic_state_machine,
                                        dummy_start_state, dummy_target_state);
  // Since the controller is aborted, will not run the controller
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  PositionYaw data_position_yaw(data.localpos.x, data.localpos.y,
                                data.localpos.z, data.rpydata.z);
  ASSERT_NE(data_position_yaw, goal);
}

TEST(PositionControlFunctorTests, TransitionGuardTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  int dummy_start_state, dummy_target_state;
  PositionYaw goal(1, 1, 1, 1);
  bsa::ReachingGoalGuard position_control_transition_guard_functor;
  bool result = position_control_transition_guard_functor(
      goal, sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_TRUE(result);
  goal = PositionYaw(1, 1, 1000, 1);
  result = position_control_transition_guard_functor(
      goal, sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_FALSE(result);
}

TEST(PositionControlFunctorTests, InternalActionTest) {
  QuadSimulator drone_hardware;
  drone_hardware.takeoff();
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
  // After running the active controller once updates quad state
  uav_system.runActiveController(HardwareType::UAV);
  // Second time updates controller status
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
