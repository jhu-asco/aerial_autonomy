#include <aerial_autonomy/robot_systems/arm_system.h>
#include <arm_parsers/generic_arm.h>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \brief Test Arm System
TEST(ArmSystemTests, Constructor) {
  ros::NodeHandle nh;
  GenericArm arm(nh);
  ASSERT_NO_THROW(new ArmSystem(arm));
}
/// \todo Add tests for arm system using a simulated arm hardware
///

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
