#include <aerial_autonomy/robot_systems/arm_system.h>
#include <arm_parsers/arm_simulator.h>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

void createSineControllerConfig(ArmSineControllerConfig *config) {
  for (int i = 0; i < 2; ++i) {
    auto p = config->mutable_joint_config()->Add();
    p->set_amplitude(1.0);
    p->set_phase((M_PI / 2.0) * (1 - i));
    p->set_frequency(0.0 + i * 1.0);
  }
}

/// \brief Test Arm System
TEST(ArmSystemTests, Constructor) {
  ASSERT_NO_THROW(new ArmSystem(std::shared_ptr<ArmParser>(new ArmSimulator)));
}

TEST(ArmSystemTests, Power) {
  ArmSystem arm_system(std::shared_ptr<ArmParser>(new ArmSimulator));
  ASSERT_FALSE(arm_system.enabled());
  arm_system.power(true);
  ASSERT_TRUE(arm_system.enabled());
}

TEST(ArmSystemTests, Command) {
  ArmSystem arm_system(std::shared_ptr<ArmParser>(new ArmSimulator));
  arm_system.power(true);
  ASSERT_FALSE(arm_system.getCommandStatus());
  arm_system.rightArm();
  ASSERT_TRUE(arm_system.getCommandStatus());
  // Test Command after powering off
  arm_system.power(false);
  arm_system.rightArm();
  ASSERT_FALSE(arm_system.getCommandStatus());
}

TEST(ArmSystemTests, JointAngles) {
  ArmSystemConfig config;
  createSineControllerConfig(config.mutable_arm_sine_controller_config());
  ArmSystem arm_system(config, std::shared_ptr<ArmParser>(new ArmSimulator));
  arm_system.power(true);
  // Choose sine controller
  arm_system.setGoal<ArmSineControllerConnector>(EmptyGoal());
  arm_system.runActiveController(ControllerGroup::Arm);
  auto joint_angles = arm_system.getJointAngles();
  ASSERT_EQ(joint_angles.size(), 2);
  ASSERT_NEAR(joint_angles[0], 1.0, 1e-2);
  ASSERT_NEAR(joint_angles[1], 0.0, 1e-2);
  arm_system.setGoal<ArmSineControllerConnector>(EmptyGoal());
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  arm_system.runActiveController(ControllerGroup::Arm);
  joint_angles = arm_system.getJointAngles();
  ASSERT_NEAR(joint_angles[0], 1.0, 1e-2);
  ASSERT_NEAR(joint_angles[1], 0.0, 1e-2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
