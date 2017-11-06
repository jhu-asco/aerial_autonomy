#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <aerial_autonomy/visual_servoing_events.h>
#include <gtest/gtest.h>
#include <iostream>
#include <typeindex>

using namespace std;

/// \brief TEST
/// All the tests are defined here
namespace uav_basic_events {
TEST(UAVEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new UAVEventManager<SampleLogicStateMachine>());
}
TEST(UAVEventManagerTest, CheckEventSet) {
  UAVEventManager<SampleLogicStateMachine> event_manager;
  std::set<std::string> event_set = event_manager.getEventSet();
  std::set<std::string> expected_set{"Land", "Takeoff", "Abort"};
  ASSERT_EQ(event_set, expected_set);
}
TEST(UAVEventManagerTest, TriggerWrongEvent) {
  UAVEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  ASSERT_FALSE(event_manager.triggerEvent("TrackROI", logic_state_machine));
}
TEST(UAVEventManagerTest, TriggerEvents) {
  UAVEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Land)));
}
}
namespace visual_servoing_events {
TEST(VisualServoingEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new VisualServoingEventManager<SampleLogicStateMachine>());
}
TEST(VisualServoingEventManagerTest, TriggerEvents) {
  VisualServoingEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("TrackROI", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(TrackROI)));
}
TEST(VisualServoingEventManagerTest, TriggerUAVEvent) {
  VisualServoingEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(uav_basic_events::Land)));
}
TEST(VisualServoingEventManagerTest, CheckEventSet) {
  VisualServoingEventManager<SampleLogicStateMachine> event_manager;
  std::set<std::string> event_set = event_manager.getEventSet();
  std::set<std::string> expected_set{"Land", "Takeoff", "Abort", "TrackROI",
                                     "GoHome"};
  ASSERT_TRUE(event_set == expected_set);
}
}

namespace pick_place_events {
TEST(PickPlaceEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new PickPlaceEventManager<SampleLogicStateMachine>());
}
TEST(PickPlaceEventManagerTest, TriggerHomeEvent) {
  PickPlaceEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("GoHome", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(visual_servoing_events::GoHome)));
}
TEST(PickPlaceEventManagerTest, TriggerPickEvent) {
  PickPlaceEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("Pick", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Pick)));
}
TEST(PickPlaceEventManagerTest, TriggerUAVEvent) {
  PickPlaceEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(uav_basic_events::Land)));
}
TEST(PickPlaceEventManagerTest, CheckEventSet) {
  PickPlaceEventManager<SampleLogicStateMachine> event_manager;
  std::set<std::string> event_set = event_manager.getEventSet();
  std::set<std::string> expected_set{
      "Land",     "Takeoff", "Abort", "Pick",           "Place", "GoHome",
      "PowerOff", "PowerOn", "Fold",  "RightAngleFold", "Grip",  "UnGrip"};
  ASSERT_TRUE(event_set == expected_set);
}
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
