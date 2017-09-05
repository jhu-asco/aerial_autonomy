#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <aerial_autonomy/state_machines/state_machine_gui_connector.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/types/velocity_yaw.h>
#include <aerial_autonomy/uav_basic_events.h>

using namespace uav_basic_events;

class StateMachineGUIConnectorTests : public ::testing::Test {
protected:
  UAVEventManager<SampleLogicStateMachine> event_manager;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine;
  ros::NodeHandle nh;
  StateMachineGUIConnector<UAVEventManager<SampleLogicStateMachine>,
                           SampleLogicStateMachine>
      state_machine_gui_connector;

  StateMachineGUIConnectorTests()
      : logic_state_machine(robot_system),
        state_machine_gui_connector(nh, event_manager, logic_state_machine) {}

  virtual ~StateMachineGUIConnectorTests() {}
};

TEST_F(StateMachineGUIConnectorTests, TriggerVelocityYaw) {
  while (!state_machine_gui_connector.isVelocityCommandConnected()) {
  }
  ros::Duration(0.2).sleep(); // Rate must be slower than test node publish rate
  ros::spinOnce();
  ASSERT_EQ(logic_state_machine.getProcessEventTypeId(),
            std::type_index(typeid(VelocityYaw)));
  VelocityYaw velocity_yaw(1, 1, 1, M_PI);
  ASSERT_EQ(logic_state_machine.getVelocityEvent(), velocity_yaw);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "state_machine_gui_connector_pose_tests");
  return RUN_ALL_TESTS();
}
