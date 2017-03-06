#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/onboard_system_handler.h>
#include <aerial_autonomy/state_machines/basic_state_machine.h>
#include <aerial_autonomy/tests/sample_parser.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace basic_events;

TEST(OnboardSystemHandlerTests, Constructor) {
  ros::NodeHandle nh;
  OnboardSystemHandlerConfig config;
  config.set_uav_parser_type("");
  OnboardSystemHandler<LogicStateMachine, BasicEventManager<LogicStateMachine>>
      onboard_system_handler(nh, config);
}

// TODO use separate NodeHandle (in same test) to publish events.  Verify
// logic state machine checks are happening and control is happening by
// introspecting parser and perhaps state machine

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "onboard_system_handler_tests");
  return RUN_ALL_TESTS();
}
