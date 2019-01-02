#include "aerial_autonomy/types/polynomial_reference_trajectory.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "polynomial_reference_config.pb.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(PolynomialReferenceTrajectory, Initialize) {
  PolynomialReferenceConfig config;
  PositionYaw start_position_yaw(0, 0, 0, 0);
  PositionYaw goal_position_yaw(0, 0, 0, 0);

  ASSERT_NO_THROW(PolynomialReferenceTrajectory(goal_position_yaw,
                                                start_position_yaw, config));
}
// Test Basis
TEST(PolynomialReferenceTrajectory, CheckBasis) {
  PolynomialReferenceConfig config;
  PositionYaw start_position_yaw(0, 0, 0, 0);
  PositionYaw goal_position_yaw(0, 0, 0, 0);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
  Eigen::MatrixXd basis = reference.findBasisMatrix(config.min_tf(), 9, 4);
  ASSERT_EQ(basis.rows(), 5);
  ASSERT_EQ(basis.cols(), 10);
  // sixth column
  ASSERT_DOUBLE_EQ(basis(0, 5), std::pow(config.min_tf(), 5));
  ASSERT_DOUBLE_EQ(basis(1, 5), 5 * std::pow(config.min_tf(), 4));
  ASSERT_DOUBLE_EQ(basis(2, 5), 5 * 4 * std::pow(config.min_tf(), 3));
  ASSERT_DOUBLE_EQ(basis(3, 5), 5 * 4 * 3 * std::pow(config.min_tf(), 2));
  ASSERT_DOUBLE_EQ(basis(4, 5), 5 * 4 * 3 * 2 * std::pow(config.min_tf(), 1));
  // second column
  ASSERT_DOUBLE_EQ(basis(0, 1), config.min_tf());
  ASSERT_DOUBLE_EQ(basis(1, 1), 1);
  ASSERT_DOUBLE_EQ(basis(2, 1), 0);
  ASSERT_DOUBLE_EQ(basis(3, 1), 0);
  ASSERT_DOUBLE_EQ(basis(4, 1), 0);
}
// Test trajectory and velocities etc
TEST(PolynomialReferenceTrajectory, CheckTrajectory) {
  PolynomialReferenceConfig config;
  PositionYaw start_position_yaw(0, 0, 0, 1);
  PositionYaw goal_position_yaw(1, 2, 3, 1.5);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
  double tf = Eigen::Vector3d(1, 2, 3).norm() / config.max_velocity();
  auto start = reference.atTime(0);
  ASSERT_NEAR(start.first[0], start_position_yaw.x, 1e-7);
  ASSERT_NEAR(start.first[1], start_position_yaw.y, 1e-7);
  ASSERT_NEAR(start.first[2], start_position_yaw.z, 1e-7);
  ASSERT_NEAR(start.first[3], 0, 1e-7);
  ASSERT_NEAR(start.first[4], 0, 1e-7);
  ASSERT_NEAR(start.first[5], start_position_yaw.yaw, 1e-7);
  ASSERT_NEAR(start.first[6], 0, 1e-7);
  ASSERT_NEAR(start.first[7], 0, 1e-7);
  ASSERT_NEAR(start.first[8], 0, 1e-7);
  ASSERT_NEAR(start.first[9], 0, 1e-7);
  ASSERT_NEAR(start.first[10], 0, 1e-7);
  ASSERT_NEAR(start.first[11], 0, 1e-7);
  ASSERT_NEAR(start.first[12], 0, 1e-7);
  ASSERT_NEAR(start.first[13], 0, 1e-7);
  ASSERT_NEAR(start.first[14], start_position_yaw.yaw, 1e-7);
  // Control
  ASSERT_DOUBLE_EQ(start.second[0], 1.0);
  ASSERT_DOUBLE_EQ(start.second[1], 0.0);
  ASSERT_DOUBLE_EQ(start.second[2], 0.0);
  ASSERT_DOUBLE_EQ(start.second[3], 0.0);
  // End
  auto end = reference.atTime(tf);
  ASSERT_NEAR(end.first[0], goal_position_yaw.x, 1e-7);
  ASSERT_NEAR(end.first[1], goal_position_yaw.y, 1e-7);
  ASSERT_NEAR(end.first[2], goal_position_yaw.z, 1e-7);
  ASSERT_NEAR(end.first[3], 0, 1e-7);
  ASSERT_NEAR(end.first[4], 0, 1e-7);
  ASSERT_NEAR(end.first[5], goal_position_yaw.yaw, 1e-7);
  ASSERT_NEAR(end.first[6], 0, 1e-7);
  ASSERT_NEAR(end.first[7], 0, 1e-7);
  ASSERT_NEAR(end.first[8], 0, 1e-7);
  ASSERT_NEAR(end.first[9], 0, 1e-7);
  ASSERT_NEAR(end.first[10], 0, 1e-7);
  ASSERT_NEAR(end.first[11], 0, 1e-7);
  ASSERT_NEAR(end.first[12], 0, 1e-7);
  ASSERT_NEAR(end.first[13], 0, 1e-7);
  ASSERT_NEAR(end.first[14], goal_position_yaw.yaw, 1e-7);
  // Control
  ASSERT_NEAR(end.second[0], 1.0, 1e-7);
  ASSERT_NEAR(end.second[1], 0.0, 1e-7);
  ASSERT_NEAR(end.second[2], 0.0, 1e-7);
  ASSERT_NEAR(end.second[3], 0.0, 1e-7);
}
TEST(PolynomialReferenceTrajectory, Noise) {
  PolynomialReferenceConfig config;
  PositionYaw start_position_yaw(0, 0, 0, -2);
  PositionYaw goal_position_yaw(1, 2, 3, 1.5);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
  double t = 1.0, a = 0.5, nu = 0.1, dt = 1e-6;
  Eigen::Vector3d noise = reference.getNoise(t, a, nu);
  Eigen::Vector3d noise_dt = reference.getNoise(t + dt, a, nu);
  Eigen::Vector3d noise_neg_dt = reference.getNoise(t - dt, a, nu);
  ASSERT_DOUBLE_EQ(noise[0], a * std::pow(sin(2 * M_PI * nu * t), 4));
  ASSERT_NEAR(noise[1], (noise_dt[0] - noise_neg_dt[0]) / (2 * dt), 1e-6);
  ASSERT_NEAR(noise[2], (noise_dt[1] - noise_neg_dt[1]) / (2 * dt), 1e-6);
}

TEST(PolynomialReferenceTrajectory, CheckTrajectoryWithNoise) {
  PolynomialReferenceConfig config;
  config.set_add_noise(true);
  PositionYaw start_position_yaw(0, 0, 0, 1);
  PositionYaw goal_position_yaw(1, 2, 3, M_PI / 2.0);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
  double tf = Eigen::Vector3d(1, 2, 3).norm() / config.max_velocity();
  double delta_t = 0.5;
  Eigen::VectorXd state1 = reference.atTime(tf + delta_t).first;
  Eigen::VectorXd state2 = reference.atTime(tf + delta_t + 1e-6).first;
  double omega = 2 * M_PI * config.forward_noise_frequency();
  double omega_z = 2 * M_PI * config.z_noise_frequency();
  double noise =
      config.forward_noise_amplitude() * std::pow(sin(omega * delta_t), 4);
  double noise_z =
      config.z_noise_amplitude() * std::pow(sin(omega_z * delta_t), 4);
  ASSERT_NEAR(state1[1], 2 + noise, 1e-8);
  ASSERT_NEAR(state1[0], 1, 1e-8);
  ASSERT_NEAR(state1[2], 3 + noise_z, 1e-8);
  // Velocity
  Eigen::Vector3d vel = (state2.head(3) - state1.head(3)) / 1e-6;
  ASSERT_NEAR(state1[6], vel[0], 1e-6);
  ASSERT_NEAR(state1[7], vel[1], 1e-6);
  ASSERT_NEAR(state1[8], vel[2], 1e-6);
}

TEST(PolynomialReferenceTrajectory, PrintTrajectory) {
  PolynomialReferenceConfig config;
  config.set_add_noise(true);
  PositionYaw start_position_yaw(0, 0, 0, -2);
  PositionYaw goal_position_yaw(1, 2, 3, 1.5);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
  std::ofstream ofile("/tmp/poly_data.csv");
  ofile << "#Time,x,y,z,roll,pitch,yaw,vx,vy,vz" << std::endl;
  for (int i = 0; i < 500; ++i) {
    double t = i * 0.02;
    auto state_control_pair = reference.atTime(t);
    Eigen::VectorXd &state = state_control_pair.first;
    ofile << t;
    for (int i = 0; i < 9; ++i) {
      ofile << "," << state[i];
    }
    ofile << std::endl;
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
