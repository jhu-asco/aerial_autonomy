#include "aerial_autonomy/types/quad_particle_reference_trajectory.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "particle_reference_config.pb.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

ParticleReferenceConfig createConfig(double kp_x, double kp_y, double kp_z,
                                     double kp_yaw) {
  ParticleReferenceConfig config;
  config.set_kp_x(kp_x);
  config.set_kp_y(kp_y);
  config.set_kp_z(kp_z);
  config.set_kp_yaw(kp_yaw);
  return config;
}

TEST(QuadParticleReferenceTrajectory, Initialize) {
  ParticleReferenceConfig config;
  PositionYaw current_position_yaw(0, 0, 0, 0);
  PositionYaw goal_position_yaw(0, 0, 0, 0);
  ASSERT_NO_THROW(
      QuadParticleTrajectory(goal_position_yaw, current_position_yaw, config));
}

TEST(QuadParticleReferenceTrajectory, Convergence) {
  ParticleReferenceConfig config;
  PositionYaw current_position_yaw(0, 0, 0, 0);
  PositionYaw goal_position_yaw(1, 2, 3, 1.57);
  QuadParticleTrajectory reference(goal_position_yaw, current_position_yaw,
                                   config);
  auto state_control_pair = reference.atTime(10); // After 10 seconds
  Eigen::VectorXd state = state_control_pair.first;
  Eigen::VectorXd control = state_control_pair.second;
  ASSERT_EQ(state.size(), 15);
  ASSERT_EQ(control.size(), 4);
  ASSERT_NEAR(control(0), 1.0, 1e-3);
  ASSERT_NEAR(control(1), 0.0, 1e-3);
  ASSERT_NEAR(control(2), 0.0, 1e-3);
  ASSERT_NEAR(control(3), 0.0, 1e-3);
  // Test state
  // pos
  ASSERT_NEAR(state(0), 1.0, 1e-3);
  ASSERT_NEAR(state(1), 2.0, 1e-3);
  ASSERT_NEAR(state(2), 3.0, 1e-3);
  // rpy
  ASSERT_NEAR(state(3), 0.0, 1e-3);
  ASSERT_NEAR(state(4), 0.0, 1e-3);
  ASSERT_NEAR(state(5), 1.57, 1e-3);
  // vel
  ASSERT_NEAR(state(6), 0.0, 1e-3);
  ASSERT_NEAR(state(7), 0.0, 1e-3);
  ASSERT_NEAR(state(8), 0.0, 1e-3);
  // rpydot
  ASSERT_NEAR(state(9), 0.0, 1e-3);
  ASSERT_NEAR(state(10), 0.0, 1e-3);
  ASSERT_NEAR(state(11), 0.0, 1e-3);
  // rpy_desired
  ASSERT_NEAR(state(12), 0.0, 1e-3);
  ASSERT_NEAR(state(13), 0.0, 1e-3);
  ASSERT_NEAR(state(14), 1.57, 1e-3);
}

TEST(QuadParticleReferenceTrajectory, StartState) {
  ParticleReferenceConfig config;
  PositionYaw current_position_yaw(0.0, 0.2, 0.3, 0);
  PositionYaw goal_position_yaw(1, 2, 3, -1.57);
  QuadParticleTrajectory reference(goal_position_yaw, current_position_yaw,
                                   config);
  auto state_control_pair = reference.atTime(0);
  Eigen::VectorXd state = state_control_pair.first;
  Eigen::VectorXd control = state_control_pair.second;
  ASSERT_EQ(control(0), 1.0);
  ASSERT_EQ(control(1), 0.0);
  ASSERT_EQ(control(2), 0.0);
  ASSERT_EQ(control(3), -config.max_yaw_rate());
  // Test state
  // pos
  ASSERT_NEAR(state(0), 0.0, 1e-3);
  ASSERT_NEAR(state(1), 0.2, 1e-3);
  ASSERT_NEAR(state(2), 0.3, 1e-3);
  // rpy
  ASSERT_EQ(state(3), 0.0);
  ASSERT_EQ(state(4), 0.0);
  ASSERT_EQ(state(5), 0.0);
  // vel
  ASSERT_EQ(state(6), config.max_velocity());
  ASSERT_EQ(state(7), config.max_velocity());
  ASSERT_EQ(state(8), config.max_velocity());
  // rpy_desired
  ASSERT_NEAR(state(12), state(3), 1e-7);
  ASSERT_NEAR(state(13), state(4), 1e-7);
  ASSERT_NEAR(state(14), state(5), 1e-7);
}

TEST(QuadParticleReferenceTrajectory, ExpState) {
  ParticleReferenceConfig config;
  config.set_max_yaw_rate(0.3);
  PositionYaw current_position_yaw(1.0, 1.8, 2.6, -1.3);
  PositionYaw goal_position_yaw(1, 2, 3, -1.57);
  QuadParticleTrajectory reference(goal_position_yaw, current_position_yaw,
                                   config);
  auto state_control_pair = reference.atTime(0);
  Eigen::VectorXd state = state_control_pair.first;
  Eigen::VectorXd control = state_control_pair.second;
  ASSERT_LT(control(0), 1.0);
  // Test state
  // pos
  ASSERT_NEAR(state(0), 1.0, 1e-3);
  ASSERT_NEAR(state(1), 1.8, 1e-3);
  ASSERT_NEAR(state(2), 2.6, 1e-3);
  // vel
  PositionYaw error = current_position_yaw - goal_position_yaw;
  ASSERT_EQ(state(6), -config.kp_x() * error.x);
  ASSERT_EQ(state(7), -config.kp_y() * error.y);
  ASSERT_EQ(state(8), -config.kp_z() * error.z);
  ASSERT_EQ(control(3), -config.kp_yaw() * error.yaw);
}

TEST(QuadParticleReferenceTrajectory, ExpStateFiniteDiff) {
  ParticleReferenceConfig config;
  config.set_max_yaw_rate(0.3);
  PositionYaw current_position_yaw(1.0, 1.8, 2.6, -1.3);
  PositionYaw goal_position_yaw(1, 2, 3, -1.57);
  QuadParticleTrajectory reference(goal_position_yaw, current_position_yaw,
                                   config);
  double dt = 1e-6;
  auto state_control_pair = reference.atTime(0);
  auto state_control_pair_dt = reference.atTime(dt);
  Eigen::Vector3d rpy_dot_expected =
      (state_control_pair_dt.first.segment<3>(3) -
       state_control_pair.first.segment<3>(3)) /
      dt;
  test_utils::ASSERT_VEC_NEAR(
      rpy_dot_expected, Eigen::Vector3d(state_control_pair.first.segment<3>(9)),
      1e-6);
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
