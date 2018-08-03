#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/roi_to_position_converter.h>
#include <arm_parsers/generic_arm.h>
#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/

using namespace quad_simulator;

TEST(UAVArmSystemTests, Constructor) {
  UAVSystemConfig config;
  auto uav_vision_system_config = config.mutable_uav_vision_system_config();
  auto uav_arm_system_config =
      uav_vision_system_config->mutable_uav_arm_system_config();
  uav_vision_system_config->mutable_camera_transform();
  uav_vision_system_config->mutable_tracking_offset_transform();
  uav_arm_system_config->mutable_arm_transform();
  uav_arm_system_config->mutable_position_controller_config();
  uav_arm_system_config->mutable_arm_system_config()->set_arm_parser_type(
      "GenericArm");
  // Fill MPC Config
  test_utils::fillMPCConfig(config);

  ASSERT_NO_THROW(new UAVArmSystem(config));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_arm_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
