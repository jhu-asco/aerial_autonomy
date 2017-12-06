#include "aerial_autonomy/robot_systems/uav_vision_system.h"

#include <gtest/gtest.h>

TEST(UAVVisionSystemTests, Constructor) {
  ros::NodeHandle nh;
  UAVSystemConfig config;
  config.mutable_uav_vision_system_config()->set_tracker_type("ROI");
  config.set_uav_parser_type("quad_simulator_parser/QuadSimParser");
  ASSERT_NO_THROW(new UAVVisionSystem(config));
}

/// \todo Matt Add more tests!

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_vision_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
