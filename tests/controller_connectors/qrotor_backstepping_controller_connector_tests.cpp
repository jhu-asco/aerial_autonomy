#include "aerial_autonomy/tests/qrotor_backstepping_controller_connector_tests.h"
#include <gtest/gtest.h>

TEST_F(QrotorBacksteppingControllerConnectorTests, InitialDisturbPos) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 3;
  double total_time = tau_vec.sum();
  Eigen::MatrixXd path(2, 3);
  path << 1, 1, 0, 0, 0, 0;
  tf::Vector3 pos_err(.05, .05, .05);
  tf::Vector3 vel_err(0, 0, 0);
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  runUntilConvergence(goal, total_time, pos_err, vel_err);
}

TEST_F(QrotorBacksteppingControllerConnectorTests, InitialDisturbVel) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 3;
  double total_time = tau_vec.sum();
  Eigen::MatrixXd path(2, 3);
  path << 1, 1, 0, 0, 0, 0;
  tf::Vector3 pos_err(0, 0, 0);
  tf::Vector3 vel_err(.05, .05, .05);
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  runUntilConvergence(goal, total_time, pos_err, vel_err);
}

TEST_F(QrotorBacksteppingControllerConnectorTests, InitialDisturbPosVel) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 3;
  double total_time = tau_vec.sum();
  Eigen::MatrixXd path(2, 3);
  path << 1, 1, 0, 0, 0, 0;
  tf::Vector3 pos_err(.05, .05, .05);
  tf::Vector3 vel_err(.05, .05, .05);
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  runUntilConvergence(goal, total_time, pos_err, vel_err);
}

TEST_F(QrotorBacksteppingControllerConnectorTests, OutOfBounds) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 6;
  double total_time = tau_vec.sum();
  Eigen::MatrixXd path(2, 3);
  path << 0, 0, 0, 3, 3, 0;
  tf::Vector3 pos_err(0, 0, -5);
  tf::Vector3 vel_err(0, 0, 3);
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  runUntilOutOfBounds(goal, total_time, pos_err, vel_err);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
