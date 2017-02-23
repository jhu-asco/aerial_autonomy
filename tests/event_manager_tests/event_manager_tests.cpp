#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/visual_servoing_events.h>
#include <gtest/gtest.h>
#include <iostream>
#include <typeindex>

using namespace std;

//// \brief Definitions
struct LogicStateMachine {
  std::type_index type_index_event_ = typeid(NULL);
  template <class Event> void process_event(Event &event) {
    type_index_event_ = typeid(event);
  }
};
////

/// \brief TEST
/// All the tests are defined here
namespace basic_events{
TEST(BasicEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new BasicEventManager<LogicStateMachine>());
}
TEST(BasicEventManagerTest, CheckEventSet) {
  BasicEventManager<LogicStateMachine> event_manager;
  std::set<std::string> event_set = event_manager.getEventSet();
  std::set<std::string> expected_set { "Land", "Takeoff", "Abort"};
  ASSERT_TRUE(event_set == expected_set);
}
TEST(BasicEventManagerTest, TriggerWrongEvent) {
  BasicEventManager<LogicStateMachine> event_manager;
  LogicStateMachine logic_state_machine;
  ASSERT_FALSE(event_manager.triggerEvent("FollowTrajectory",logic_state_machine));
}
TEST(BasicEventManagerTest, TriggerEvents) {
  BasicEventManager<LogicStateMachine> event_manager;
  LogicStateMachine logic_state_machine;
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.type_index_event_, std::type_index(typeid(Land)));
}
}
namespace visual_servoing_events{
TEST(VisualServoingEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new VisualServoingEventManager<LogicStateMachine>());
}
TEST(VisualServoingEventManagerTest, TriggerEvents) {
  VisualServoingEventManager<LogicStateMachine> event_manager;
  LogicStateMachine logic_state_machine;
  event_manager.triggerEvent("FollowTrajectory", logic_state_machine);
  ASSERT_EQ(logic_state_machine.type_index_event_, std::type_index(typeid(FollowTrajectory)));
}
TEST(VisualServoingEventManagerTest, TriggerBasicEvent) {
  VisualServoingEventManager<LogicStateMachine> event_manager;
  LogicStateMachine logic_state_machine;
  event_manager.triggerEvent("Land", logic_state_machine);
  ASSERT_EQ(logic_state_machine.type_index_event_, std::type_index(typeid(basic_events::Land)));
}
TEST(VisualServoingEventManagerTest, CheckEventSet) {
  VisualServoingEventManager<LogicStateMachine> event_manager;
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
