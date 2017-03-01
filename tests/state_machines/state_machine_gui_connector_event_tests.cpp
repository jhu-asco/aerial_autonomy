#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/state_machines/state_machine_gui_connector.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>

using namespace basic_events;

class StateMachineGUIConnectorTests : public ::testing::Test {
protected:
  BasicEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine;
  ros::NodeHandle nh;
  StateMachineGUIConnector<BasicEventManager<SampleLogicStateMachine>,
                           SampleLogicStateMachine>
      state_machine_gui_connector;

  StateMachineGUIConnectorTests()
      : logic_state_machine(robot_system),
        state_machine_gui_connector(nh, event_manager, logic_state_machine) {}

  virtual ~StateMachineGUIConnectorTests() {}
};

TEST_F(StateMachineGUIConnectorTests, Constructor) {}

TEST_F(StateMachineGUIConnectorTests, TriggerEvent) {
  while (!state_machine_gui_connector.isEventManagerConnected()) {
    ros::Duration(0.2)
        .sleep(); // Rate must be slower than test node publish rate
    ros::spinOnce();
  }
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(Land)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "state_machine_gui_connector_event_tests");
  return RUN_ALL_TESTS();
}
