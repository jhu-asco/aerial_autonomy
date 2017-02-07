#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/visual_servoing_events.h>
#include <gtest/gtest.h>
#include <iostream>
#include <typeinfo>

using namespace std;

//// \brief Definitions
struct LogicStateMachine {
  template <class Event> void process_event(Event &event) {
    std::cout << "Event type : " << typeid(event).name() << std::endl;
  }
};
////

/// \brief TEST
/// All the tests are defined here
namespace basic_events{
TEST(BasicEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new BasicEventManager<LogicStateMachine>());
}
TEST(BasicEventManagerTest, PrintEventMap) {
  BasicEventManager<LogicStateMachine> event_manager;
  ASSERT_NO_THROW(event_manager.printEventList());
}
TEST(BasicEventManagerTest, TriggerEvents) {
  BasicEventManager<LogicStateMachine> event_manager;
  LogicStateMachine logic_state_machine;
  event_manager.triggerEvent("Land", logic_state_machine);
  event_manager.triggerEvent("Takeoff", logic_state_machine);
  event_manager.triggerEvent("Abort", logic_state_machine);
}
}
namespace visual_servoing_events{
TEST(VisualServoingEventManagerTest, InstantiateManager) {
  ASSERT_NO_THROW(new VisualServoingEventManager<LogicStateMachine>());
}
TEST(VisualServoingEventManagerTest, TriggerEvents) {
  VisualServoingEventManager<LogicStateMachine> event_manager;
  LogicStateMachine logic_state_machine;
  event_manager.triggerEvent("Land", logic_state_machine);
  event_manager.triggerEvent("Takeoff", logic_state_machine);
  event_manager.triggerEvent("Abort", logic_state_machine);
  event_manager.triggerEvent("FollowTrajectory", logic_state_machine);
}
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
