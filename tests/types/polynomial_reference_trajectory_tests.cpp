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
  config.set_tf(2);
  PositionYaw start_position_yaw(0, 0, 0, 0);
  PositionYaw goal_position_yaw(0, 0, 0, 0);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
  Eigen::MatrixXd basis = reference.findBasisMatrix(config.tf(), 9, 4);
  ASSERT_EQ(basis.rows(), 5);
  ASSERT_EQ(basis.cols(), 10);
  // sixth column
  ASSERT_DOUBLE_EQ(basis(0, 5), std::pow(config.tf(), 5));
  ASSERT_DOUBLE_EQ(basis(1, 5), 5 * std::pow(config.tf(), 4));
  ASSERT_DOUBLE_EQ(basis(2, 5), 5 * 4 * std::pow(config.tf(), 3));
  ASSERT_DOUBLE_EQ(basis(3, 5), 5 * 4 * 3 * std::pow(config.tf(), 2));
  ASSERT_DOUBLE_EQ(basis(4, 5), 5 * 4 * 3 * 2 * std::pow(config.tf(), 1));
  // second column
  ASSERT_DOUBLE_EQ(basis(0, 1), config.tf());
  ASSERT_DOUBLE_EQ(basis(1, 1), 1);
  ASSERT_DOUBLE_EQ(basis(2, 1), 0);
  ASSERT_DOUBLE_EQ(basis(3, 1), 0);
  ASSERT_DOUBLE_EQ(basis(4, 1), 0);
}
// Test trajectory and velocities etc
TEST(PolynomialReferenceTrajectory, CheckTrajectory) {
  PolynomialReferenceConfig config;
  config.set_tf(5);
  PositionYaw start_position_yaw(0, 0, 0, 1);
  PositionYaw goal_position_yaw(1, 2, 3, 1.5);
  PolynomialReferenceTrajectory reference(goal_position_yaw, start_position_yaw,
                                          config);
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
  auto end = reference.atTime(5);
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
