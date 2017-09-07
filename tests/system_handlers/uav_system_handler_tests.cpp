#include <aerial_autonomy/state_machines/uav_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_system_handler.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

using namespace uav_basic_events;

class UAVSystemHandlerTests : public ::testing::Test,
                              public test_utils::BaseTestPubSubs {
public:
  UAVSystemHandlerTests() : BaseTestPubSubs() {
    // Configure system
    UAVSystemHandlerConfig uav_system_handler_config;
    uav_system_handler_config.set_uav_parser_type(
        "quad_simulator_parser/QuadSimParser");
    uav_system_handler_config.set_sensor_type("Guidance");
    uav_system_handler_config.mutable_uav_system_config()
        ->set_minimum_takeoff_height(0.4);

    uav_system_handler_.reset(
        new UAVSystemHandler<UAVStateMachine, UAVEventManager<UAVStateMachine>>(
            uav_system_handler_config));
    ros::spinOnce();
  }

  const unsigned long timeout_wait =
      20; ///< Timeout for ros topic wait in seconds
  std::unique_ptr<UAVSystemHandler<UAVStateMachine,
                                   UAVEventManager<UAVStateMachine>>>
      uav_system_handler_; ///< system contains robot system, state machine
};

TEST_F(UAVSystemHandlerTests, Constructor) {}

TEST_F(UAVSystemHandlerTests, TestConnections) {
  while (!uav_system_handler_->isConnected()) {
  }
  SUCCEED();
}

TEST_F(UAVSystemHandlerTests, ProcessEvents) {
  while (!uav_system_handler_->isConnected()) {
  }
  auto armed_true_fun = [=]() {
    ros::spinOnce();
    return uav_system_handler_->getUAVData().armed;
  };
  // Check takeoff works
  publishEvent("Takeoff");
  ASSERT_TRUE(test_utils::waitUntilTrue()(armed_true_fun,
                                          std::chrono::seconds(timeout_wait)));
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.5);
  // Check subsequent event works
  publishEvent("Land");
  ASSERT_FALSE(test_utils::waitUntilFalse()(
      armed_true_fun, std::chrono::seconds(timeout_wait)));
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.0);
}

TEST_F(UAVSystemHandlerTests, ProcessPoseCommand) {
  while (!uav_system_handler_->isConnected()) {
  }
  auto armed_true_fun = [=]() {
    ros::spinOnce();
    return uav_system_handler_->getUAVData().armed;
  };
  // Check pose command works
  publishEvent("Takeoff");
  ASSERT_TRUE(test_utils::waitUntilTrue()(armed_true_fun,
                                          std::chrono::seconds(timeout_wait)));
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.5);
  PositionYaw pose_command(1, 2, 3, 0);
  publishPoseCommand(pose_command);
  ASSERT_TRUE(test_utils::waitUntilTrue()(
      [=]() {
        ros::spinOnce();
        return pose_command ==
               getPositionYaw(uav_system_handler_->getUAVData());
      },
      std::chrono::seconds(timeout_wait)));
}

TEST_F(UAVSystemHandlerTests, ReceiveStatus) {
  while (!isStatusConnected())
    ;
  ASSERT_FALSE(test_utils::waitUntilFalse()(
      [=]() {
        ros::spinOnce();
        return status_.empty();
      },
      std::chrono::seconds(timeout_wait)));
}

TEST_F(UAVSystemHandlerTests, ) {}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "uav_system_handler_tests");
  return RUN_ALL_TESTS();
}
