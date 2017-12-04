#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/state_machines/pick_place_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_arm_system_handler.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

using namespace uav_basic_events;

class UAVArmSystemHandlerTests : public ::testing::Test,
                                 public test_utils::BaseTestPubSubs {
public:
  UAVArmSystemHandlerTests() : BaseTestPubSubs() {
    // Configure system
    UAVSystemHandlerConfig uav_system_handler_config;
    BaseStateMachineConfig state_machine_config;
    uav_system_handler_config.mutable_uav_system_config()
        ->set_minimum_takeoff_height(0.4);
    auto uav_config = uav_system_handler_config.mutable_uav_system_config();
    uav_config->set_uav_parser_type("quad_simulator_parser/QuadSimParser");
    uav_config->mutable_uav_vision_system_config()
        ->mutable_uav_arm_system_config()
        ->mutable_arm_system_config()
        ->set_arm_parser_type("ArmSimulator");
    // Position controller params
    auto pos_controller_config =
        uav_config->mutable_rpyt_based_position_controller_config()
            ->mutable_velocity_based_position_controller_config();
    pos_controller_config->set_position_gain(1.);
    pos_controller_config->set_z_gain(1.);
    pos_controller_config->set_yaw_gain(1.);
    pos_controller_config->set_max_velocity(2.);
    pos_controller_config->set_max_yaw_rate(5.);
    auto goal_position_tolerance =
        pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    pos_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(0.1);
    goal_position_tolerance->set_x(0.1);
    goal_position_tolerance->set_y(0.1);
    goal_position_tolerance->set_z(0.1);
    auto rpyt_vel_controller_tol =
        uav_config->mutable_rpyt_based_position_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    rpyt_vel_controller_tol->set_vx(0.1);
    rpyt_vel_controller_tol->set_vy(0.1);
    rpyt_vel_controller_tol->set_vz(0.1);

    uav_system_handler_.reset(
        new UAVArmSystemHandler<
            PickPlaceStateMachine,
            pick_place_events::PickPlaceEventManager<PickPlaceStateMachine>>(
            uav_system_handler_config, state_machine_config));
    ros::spinOnce();
  }

  const unsigned long timeout_wait =
      20; ///< Timeout for ros topic wait in seconds
  std::unique_ptr<UAVArmSystemHandler<
      PickPlaceStateMachine,
      pick_place_events::PickPlaceEventManager<PickPlaceStateMachine>>>
      uav_system_handler_; ///< system contains robot system, state machine
};

TEST_F(UAVArmSystemHandlerTests, Constructor) {}

TEST_F(UAVArmSystemHandlerTests, TestConnections) {
  while (!uav_system_handler_->isConnected()) {
  }
  SUCCEED();
}

TEST_F(UAVArmSystemHandlerTests, ProcessEvents) {
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

TEST_F(UAVArmSystemHandlerTests, ProcessPoseCommand) {
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
        return (pose_command -
                getPositionYaw(uav_system_handler_->getUAVData()))
                   .position()
                   .norm() < 0.1;
      },
      std::chrono::seconds(timeout_wait)));
}

TEST_F(UAVArmSystemHandlerTests, ReceiveStatus) {
  while (!isStatusConnected())
    ;
  ASSERT_FALSE(test_utils::waitUntilFalse()(
      [=]() {
        ros::spinOnce();
        return status_.empty();
      },
      std::chrono::seconds(timeout_wait)));
}

/// \todo Matt Add pick and place tests

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "uav_arm_system_handler_tests");
  return RUN_ALL_TESTS();
}
