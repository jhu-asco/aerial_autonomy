#include <aerial_autonomy/robot_systems/arm_system.h>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \brief Test Arm System
TEST(ArmSystemTests, Constructor) {
  ros::NodeHandle nh;
  ASSERT_NO_THROW(new ArmSystem(nh));
}
/// \todo Add tests for arm system using a simulated arm hardware
///

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
