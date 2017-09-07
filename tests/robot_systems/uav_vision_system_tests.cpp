#include "aerial_autonomy/robot_systems/uav_vision_system.h"
#include "aerial_autonomy/trackers/roi_to_position_converter.h"

#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

TEST(UAVVisionSystemTests, Constructor) {
  QuadSimulator drone_hardware;
  ros::NodeHandle nh;
  RoiToPositionConverter roi_to_position_converter(nh);
  UAVSystemConfig config;
  auto uav_vision_system_config = config.mutable_uav_vision_system_config();
  for (int i = 0; i < 6; ++i) {
    uav_vision_system_config->add_camera_transform(0.0);
  }
  for (int i = 0; i < 6; ++i) {
    uav_vision_system_config->add_tracking_offset_transform(0.0);
  }
  ASSERT_NO_THROW(
      new UAVVisionSystem(roi_to_position_converter, drone_hardware, config));
}

/// \todo Matt Add more tests!

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_vision_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
