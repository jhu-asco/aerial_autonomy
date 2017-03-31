#include "aerial_autonomy/robot_systems/uav_vision_system.h"

#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

TEST(UAVSystemTests, Constructor) {
  QuadSimulator drone_hardware;
  ros::NodeHandle nh;
  UAVSystemConfig config;
  ASSERT_NO_THROW(new UAVVisionSystem(nh, drone_hardware, config));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_vision_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
