#include <aerial_autonomy/robot_systems/arm_system.h>
#include <arm_parsers/arm_simulator.h>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

/// \brief Test Arm System
TEST(ArmSystemTests, Constructor) {
  ArmSimulator arm;
  ASSERT_NO_THROW(new ArmSystem(arm));
}

TEST(ArmSystemTests, Power) {
  ArmSimulator arm;
  ArmSystem arm_system(arm);
  ASSERT_FALSE(arm_system.enabled());
  arm_system.power(true);
  ASSERT_TRUE(arm_system.enabled());
}

TEST(ArmSystemTests, Command) {
  ArmSimulator arm;
  ArmSystem arm_system(arm);
  arm_system.power(true);
  ASSERT_FALSE(arm_system.getCommandStatus());
  arm_system.rightArm();
  ASSERT_TRUE(arm_system.getCommandStatus());
  // Test Command after powering off
  arm_system.power(false);
  arm_system.rightArm();
  ASSERT_FALSE(arm_system.getCommandStatus());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
