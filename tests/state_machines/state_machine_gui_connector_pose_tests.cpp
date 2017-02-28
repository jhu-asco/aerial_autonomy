#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/state_machines/state_machine_gui_connector.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/types/position_yaw.h>

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

TEST_F(StateMachineGUIConnectorTests, TriggerPose) {
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(PositionYaw)));
  PositionYaw pose(1, 2, 3, 0);
  ASSERT_EQ(logic_state_machine.getPoseEvent(), pose);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "state_machine_gui_connector_pose_tests");
  return RUN_ALL_TESTS();
}
