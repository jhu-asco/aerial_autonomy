#include <aerial_autonomy/robot_systems/uav_arm_system.h>
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
  uav_vision_system_config->mutable_uav_arm_system_config()
      ->mutable_position_controller_config();
  QuadSimulator drone_hardware;
  ros::NodeHandle nh;
  GenericArm arm(nh);
  RoiToPositionConverter roi_to_position_converter(nh);

  ASSERT_NO_THROW(
      new UAVArmSystem(roi_to_position_converter, drone_hardware, arm, config));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_arm_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
