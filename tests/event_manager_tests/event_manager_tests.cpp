#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/visual_servoing_events.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <gtest/gtest.h>
#include <iostream>
#include <typeindex>

using namespace std;

/// \brief TEST
/// All the tests are defined here
namespace basic_events{
TEST(BasicEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new BasicEventManager<SampleLogicStateMachine>());
}
TEST(BasicEventManagerTest, CheckEventSet) {
  BasicEventManager<SampleLogicStateMachine> event_manager;
  std::set<std::string> event_set = event_manager.getEventSet();
  std::set<std::string> expected_set { "Land", "Takeoff", "Abort"};
  ASSERT_TRUE(event_set == expected_set);
}
TEST(BasicEventManagerTest, TriggerWrongEvent) {
  BasicEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  ASSERT_FALSE(event_manager.triggerEvent("FollowTrajectory",logic_state_machine));
}
TEST(BasicEventManagerTest, TriggerEvents) {
  BasicEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Land)));
}
}
namespace visual_servoing_events{
TEST(VisualServoingEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new VisualServoingEventManager<SampleLogicStateMachine>());
}
TEST(VisualServoingEventManagerTest, TriggerEvents) {
  VisualServoingEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("FollowTrajectory", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(FollowTrajectory)));
}
TEST(VisualServoingEventManagerTest, TriggerBasicEvent) {
  VisualServoingEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(basic_events::Land)));
}
TEST(VisualServoingEventManagerTest, CheckEventSet) {
  VisualServoingEventManager<SampleLogicStateMachine> event_manager;
  std::set<std::string> event_set = event_manager.getEventSet();
  std::set<std::string> expected_set { "Land", "Takeoff", "Abort", "FollowTrajectory"};
  ASSERT_TRUE(event_set == expected_set);
}
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
