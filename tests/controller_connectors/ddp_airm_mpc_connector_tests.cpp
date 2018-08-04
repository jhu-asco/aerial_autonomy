#include "aerial_autonomy/tests/ddp_airm_mpc_connector_tests.h"
#include "aerial_autonomy/common/conversions.h"
#include <gtest/gtest.h>

TEST_F(MPCControllerAirmConnectorTests, OmegaToRpyDot) {
  Eigen::Vector3d omega(1, 0, 0);
  Eigen::Vector3d rpy(1, 1.2, -1); // Random values
  Eigen::Vector3d rpydot = controller_connector_->omegaToRpyDot(omega, rpy);
  test_utils::ASSERT_VEC_NEAR(rpydot, Eigen::Vector3d(1, 0, 0));
  // rpy = 0, rpydot = omega
  omega = Eigen::Vector3d(1, 2, 3);
  rpy = Eigen::Vector3d(0, 0, 0);
  rpydot = controller_connector_->omegaToRpyDot(omega, rpy);
  test_utils::ASSERT_VEC_NEAR(rpydot, omega);
}

TEST_F(MPCControllerAirmConnectorTests, OmegaToRpyDotSingularity) {
  Eigen::Vector3d omega(1, 1.2, 2.3);
  Eigen::Vector3d rpy(1, -M_PI / 2.0, -1); // Pitch equal to pi/2
  Eigen::Vector3d rpydot = controller_connector_->omegaToRpyDot(omega, rpy);
  Eigen::Vector3d rpy_clamped(1, -0.9 * (M_PI / 2.0), -1);
  Eigen::Vector3d rpydot_clamped =
      controller_connector_->omegaToRpyDot(omega, rpy);
  test_utils::ASSERT_VEC_NEAR(rpydot, rpydot_clamped);
}

TEST_F(MPCControllerAirmConnectorTests, OmegaToRpyDotFiniteDiff) {
  Eigen::Vector3d omega(1, 2, 3);
  Eigen::Vector3d rpy(1, 1.2, -1); // Random values
  tf::Transform R;
  conversions::transformRPYToTf(rpy[0], rpy[1], rpy[2], R);
  double dt = 1e-6;
  tf::Vector3 axis(omega[0], omega[1], omega[2]);
  double angle = axis.length() * dt;
  axis.normalize();
  tf::Transform dR(tf::Quaternion(axis, angle));
  tf::Transform Rn = R * dR;
  Eigen::Vector3d rpy_next = conversions::transformTfToRPY(Rn);
  Eigen::Vector3d rpydot_fd = (1.0 / dt) * (rpy_next - rpy);

  Eigen::Vector3d rpydot = controller_connector_->omegaToRpyDot(omega, rpy);
  test_utils::ASSERT_VEC_NEAR(rpydot, rpydot_fd, 1e-3);
}

TEST_F(MPCControllerAirmConnectorTests, GoalIsStart) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(0, 0, 0, 0),
                      {-0.8, 0}, {-0.8, 0}, false);
}

TEST_F(MPCControllerAirmConnectorTests, ConvergenceZeroStartNoYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(1, -1, 1, 0),
                      {-0.8, 0}, {-0.8, 0});
}

TEST_F(MPCControllerAirmConnectorTests, ConvergenceZeroStartPosYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(-0.1, -1, 0.1, 0.1),
                      {-0.8, 0}, {-0.8, 0});
}

TEST_F(MPCControllerAirmConnectorTests, ConvergenceZeroStartNegYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(1, -1, -1, -0.5),
                      {-0.8, 0}, {-0.8, 0});
}

TEST_F(MPCControllerAirmConnectorTests, ConvergenceNonZeroStartNoYaw) {
  runUntilConvergence(PositionYaw(-1, 0.3, 2, 0), PositionYaw(-0.5, 0.5, 1, 0),
                      {-0.8, 0}, {-0.8, 0});
}

TEST_F(MPCControllerAirmConnectorTests, ConvergenceNonZeroStartPosYaw) {
  runUntilConvergence(PositionYaw(-2.2, -1, 3.2, -0.1),
                      PositionYaw(-2, -1.5, 3.5, 0.5), {-0.8, 0}, {-0.8, 0});
}

TEST_F(MPCControllerAirmConnectorTests, ConvergenceNonZeroStartNegYaw) {
  runUntilConvergence(PositionYaw(-.1, 0.2, 1, 0.4),
                      PositionYaw(1, -1, -1, -0.5), {-0.8, 0}, {-0.8, 0});
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
