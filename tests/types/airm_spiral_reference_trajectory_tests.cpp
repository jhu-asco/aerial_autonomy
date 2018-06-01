#include "aerial_autonomy/types/spiral_reference_trajectory.h"
#include "arm_sine_controller_config.pb.h"
#include <gtest/gtest.h>
#include <memory>

class AirmSpiralReferenceTrajectoryTests : public ::testing::Test {
public:
  AirmSpiralReferenceTrajectoryTests()
      : current_position(Eigen::Vector3d(1.0, 2.0, 3.0)), current_yaw(0.5),
        kt(0.15) {
    createConfig(arm_config, quad_config);
    reference_trajectory.reset(new SpiralReferenceTrajectory(
        quad_config, arm_config, current_position, current_yaw, kt));
  }

protected:
  ArmSineControllerConfig arm_config;
  SpiralReferenceTrajectoryConfig quad_config;
  std::shared_ptr<SpiralReferenceTrajectory> reference_trajectory;
  Eigen::Vector3d current_position;
  double current_yaw;
  double kt;
  void createConfig(ArmSineControllerConfig &arm_config,
                    SpiralReferenceTrajectoryConfig &quad_config) {
    auto joint1_config = arm_config.add_joint_config();
    auto joint2_config = arm_config.add_joint_config();
    joint1_config->set_amplitude(1.0);
    joint2_config->set_amplitude(0.5);

    joint1_config->set_frequency(10.0);
    joint2_config->set_frequency(1.0);

    joint1_config->set_offset(1.0);
    joint2_config->set_offset(2.0);

    joint1_config->set_phase(0.0);
    joint2_config->set_phase(M_PI / 2.0);
    // Quad settings
    quad_config.set_frequency(5.0);
    quad_config.set_phase(0.0);

    quad_config.set_radiusx(1.0);
    quad_config.set_radiusy(0.5);

    quad_config.set_velocity_z(1.0);
    quad_config.set_frequency_z(1.0);

    quad_config.set_amplitude_yaw(1.0);
    quad_config.set_frequency_yaw(0.1);
  }
};

TEST(AirmSpiralReferenceTrajectory, Initialize) {
  ArmSineControllerConfig arm_config;
  arm_config.add_joint_config();
  arm_config.add_joint_config();
  SpiralReferenceTrajectoryConfig quad_config;
  SpiralReferenceTrajectory reference_trajectory(
      quad_config, arm_config, Eigen::Vector3d::Zero(), 0.0, 0.15);
}

TEST_F(AirmSpiralReferenceTrajectoryTests, ZeroTime) {
  auto state_control_pair = reference_trajectory->atTime(0.0);
  auto state = state_control_pair.first;
  auto control = state_control_pair.second;
  double omega = quad_config.frequency() * 2 * M_PI;
  ASSERT_EQ(state[0], current_position[0]);
  ASSERT_EQ(state[1], current_position[1] + quad_config.radiusy());
  ASSERT_EQ(state[2], current_position[2]);
  ASSERT_EQ(state[5], current_yaw);
  ASSERT_EQ(state[6], quad_config.radiusx() * omega);
  ASSERT_EQ(state[7], 0.0);
  ASSERT_EQ(state[8], quad_config.velocity_z());
  // Joints
  ASSERT_EQ(state[15], 1.0);
  ASSERT_EQ(state[16], 2.0 + 0.5);
  ASSERT_EQ(state[17], 2 * M_PI * 10.0);
  ASSERT_NEAR(state[18], 0.0, 1e-15);
  ASSERT_EQ(state[19], 1.0); // Same as angles
  ASSERT_EQ(state[20], 2.0 + 0.5);
  ASSERT_EQ(control[0], 9.81 / kt);
  // Desired rate
  ASSERT_EQ(control[1], state[9]);
  ASSERT_EQ(control[2], state[10]);
  ASSERT_EQ(control[3], state[11]);
  // Joint
  ASSERT_EQ(control[4], state[17]);
  ASSERT_EQ(control[5], state[18]);
}

TEST_F(AirmSpiralReferenceTrajectoryTests, Period) {
  auto state_control_pair = reference_trajectory->atTime(1.0);
  auto state = state_control_pair.first;
  auto control = state_control_pair.second;
  double omega = quad_config.frequency() * 2 * M_PI;
  double omega_yaw = 2 * M_PI * quad_config.frequency_yaw();
  ASSERT_NEAR(state[0], current_position[0], 1e-14);
  ASSERT_NEAR(state[1], current_position[1] + quad_config.radiusy(), 1e-14);
  ASSERT_NEAR(state[2], current_position[2] + 1.0, 1e-14);
  ASSERT_EQ(state[5],
            current_yaw + quad_config.amplitude_yaw() * sin(2 * M_PI * 0.1));
  ASSERT_NEAR(state[6], quad_config.radiusx() * omega, 1e-14);
  ASSERT_NEAR(state[7], 0.0, 1e-13);
  ASSERT_NEAR(state[8], -quad_config.velocity_z(), 1e-14);
  ASSERT_EQ(state[11],
            quad_config.amplitude_yaw() * omega_yaw * cos(omega_yaw));
  // Joints
  ASSERT_NEAR(state[15], 1.0, 1e-10);
  ASSERT_NEAR(state[16], 2.0 + 0.5, 1e-10);
  ASSERT_NEAR(state[17], 2 * M_PI * 10.0, 1e-10);
  ASSERT_NEAR(state[18], 0.0, 1e-15);
  ASSERT_NEAR(state[19], 1.0, 1e-10); // Same as angles
  ASSERT_NEAR(state[20], 2.0 + 0.5, 1e-10);
}
// Check rp
TEST_F(AirmSpiralReferenceTrajectoryTests, ZeroRollPitch) {
  double roll, pitch, yaw;
  yaw = 0.0;
  reference_trajectory->getRP(roll, pitch, yaw, Eigen::Vector3d(0, 0, 5));
  ASSERT_EQ(roll, 0.0);
  ASSERT_EQ(pitch, 0.0);
}

TEST_F(AirmSpiralReferenceTrajectoryTests, ZeroRollNinetyPitch) {
  double roll, pitch, yaw;
  yaw = 0.0;
  reference_trajectory->getRP(roll, pitch, yaw, Eigen::Vector3d(5, 0, 0));
  ASSERT_EQ(roll, 0.0);
  ASSERT_EQ(pitch, M_PI / 2);
  reference_trajectory->getRP(roll, pitch, yaw, Eigen::Vector3d(-5, 0, 0));
  ASSERT_EQ(roll, 0.0);
  ASSERT_EQ(pitch, -M_PI / 2);
  yaw = 0.5;
  reference_trajectory->getRP(roll, pitch, yaw,
                              Eigen::Vector3d(5 * cos(yaw), 5 * sin(yaw), 0));
  ASSERT_EQ(roll, 0.0);
  ASSERT_EQ(pitch, M_PI / 2);
}

TEST_F(AirmSpiralReferenceTrajectoryTests, Singularity) {
  double roll, pitch, yaw;
  yaw = 0.0;
  reference_trajectory->getRP(roll, pitch, yaw, Eigen::Vector3d(0, 5, 0));
  ASSERT_EQ(roll, -M_PI / 2.0);
  ASSERT_EQ(pitch, 0.0);
  reference_trajectory->getRP(roll, pitch, yaw, Eigen::Vector3d(0, -5, 0));
  ASSERT_EQ(roll, M_PI / 2.0);
  ASSERT_EQ(pitch, 0.0);
  reference_trajectory->getRP(roll, pitch, yaw, Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(roll, 0.0);
  ASSERT_EQ(pitch, 0.0);
}
// Check zero frequency
TEST_F(AirmSpiralReferenceTrajectoryTests, ZeroFrequency) {
  quad_config.set_frequency(0.0);
  quad_config.set_velocity_z(0.0);
  quad_config.set_frequency_yaw(0.0);
  arm_config.mutable_joint_config(0)->set_frequency(0.0);
  arm_config.mutable_joint_config(1)->set_frequency(0.0);
  reference_trajectory.reset(new SpiralReferenceTrajectory(
      quad_config, arm_config, current_position, current_yaw, kt));
  auto state_control_pair = reference_trajectory->atTime(0.0);
  auto state_control_pair_2 = reference_trajectory->atTime(5.0);
  for (int i = 0; i < 21; ++i) {
    ASSERT_EQ(state_control_pair.first[i], state_control_pair_2.first[i])
        << "i: " << i;
  }
  for (int i = 0; i < 6; ++i) {
    ASSERT_EQ(state_control_pair.second[i], state_control_pair_2.second[i])
        << "i: " << i;
  }
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(state_control_pair.first[9 + i], 0.0);
  }
}

TEST_F(AirmSpiralReferenceTrajectoryTests, FiniteDifference) {
  double dt = 1e-6;
  auto state_control_pair = reference_trajectory->atTime(0.0);
  auto state_control_pair_2 = reference_trajectory->atTime(dt);
  Eigen::Vector3d predicted_rpy_dot =
      (state_control_pair_2.first.segment<3>(3) -
       state_control_pair.first.segment<3>(3)) /
      dt;
  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(predicted_rpy_dot[i], state_control_pair.first[9 + i], 1e-6);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
