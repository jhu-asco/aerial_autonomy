#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "minimum_snap_reference_trajectory_config.pb.h"
#include <aerial_autonomy/types/minimum_snap_reference_trajectory.h>
#include <chrono>
#include <gtest/gtest.h>

void ASSERT_STATES_NEAR(const Eigen::MatrixXd &state1,
                        const std::pair<ParticleState, Snap> &state2,
                        double tol = 1e-7) {
  ASSERT_NEAR(state1(0, 0), std::get<0>(state2).p.x, tol);
  ASSERT_NEAR(state1(0, 1), std::get<0>(state2).p.y, tol);
  ASSERT_NEAR(state1(0, 2), std::get<0>(state2).p.z, tol);

  ASSERT_NEAR(state1(1, 0), std::get<0>(state2).v.x, tol);
  ASSERT_NEAR(state1(1, 1), std::get<0>(state2).v.y, tol);
  ASSERT_NEAR(state1(1, 2), std::get<0>(state2).v.z, tol);

  ASSERT_NEAR(state1(2, 0), std::get<0>(state2).a.x, tol);
  ASSERT_NEAR(state1(2, 1), std::get<0>(state2).a.y, tol);
  ASSERT_NEAR(state1(2, 2), std::get<0>(state2).a.z, tol);

  ASSERT_NEAR(state1(3, 0), std::get<0>(state2).j.x, tol);
  ASSERT_NEAR(state1(3, 1), std::get<0>(state2).j.y, tol);
  ASSERT_NEAR(state1(3, 2), std::get<0>(state2).j.z, tol);

  ASSERT_NEAR(state1(4, 0), std::get<1>(state2).x, tol);
  ASSERT_NEAR(state1(4, 1), std::get<1>(state2).y, tol);
  ASSERT_NEAR(state1(4, 2), std::get<1>(state2).z, tol);
}

void setWaypoint(config::PositionYaw *way_point, double x, double y, double z,
                 double yaw) {
  way_point->mutable_position()->set_x(x);
  way_point->mutable_position()->set_y(y);
  way_point->mutable_position()->set_z(z);
  way_point->set_yaw(yaw);
}

TEST(MinimumSnapReferenceTrajectory, Constructor) {
  int r = 4;
  Eigen::VectorXd tau_vec(2);
  tau_vec << 1, 1;
  Eigen::MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  MinimumSnapReferenceTrajectory ref_first(r, tau_vec, path);

  MinimumSnapReferenceTrajectoryConfig ref_config;
  ref_config.add_tau_vec(1);
  ref_config.add_tau_vec(1);
  auto waypoint_config =
      ref_config.mutable_following_waypoint_sequence_config();
  setWaypoint(waypoint_config->add_way_points(), 0, 0, 0, 0);
  setWaypoint(waypoint_config->add_way_points(), 1, 1, 1, 0);
  setWaypoint(waypoint_config->add_way_points(), 2, 2, 2, 0);
  MinimumSnapReferenceTrajectory ref_second(ref_config);
  test_utils::ASSERT_EIGENMAT_NEAR(ref_first.getP(), ref_second.getP());
}

TEST(MinimumSnapReferenceTrajectory, DerOrderNegative) {
  int r = -1;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 1;
  Eigen::MatrixXd path(2, 3);
  path << 0, 0, 0, 1, 1, 1;
  ASSERT_THROW(MinimumSnapReferenceTrajectory(r, tau_vec, path),
               std::invalid_argument);
}

TEST(MinimumSnapReferenceTrajectory, TauVecNegative) {
  int r = 4;
  Eigen::VectorXd tau_vec(2);
  tau_vec << -1, 1;
  Eigen::MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  ASSERT_THROW(MinimumSnapReferenceTrajectory(r, tau_vec, path),
               std::invalid_argument);
}

TEST(MinimumSnapReferenceTrajectory, PathColTooSmall) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 1;
  Eigen::MatrixXd path(2, 2);
  path << 0, 0, 1, 1;
  ASSERT_THROW(MinimumSnapReferenceTrajectory(r, tau_vec, path),
               std::length_error);
}

TEST(MinimumSnapReferenceTrajectory, PathColTooLarge) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 1;
  Eigen::MatrixXd path(2, 4);
  path << 0, 0, 0, 0, 1, 1, 1, 1;
  ASSERT_THROW(MinimumSnapReferenceTrajectory(r, tau_vec, path),
               std::length_error);
}

TEST(MinimumSnapReferenceTrajectory, PathRowTooSmall) {
  int r = 4;
  Eigen::VectorXd tau_vec(3);
  tau_vec << 1, 1, 1;
  Eigen::MatrixXd path(2, 3);
  path << 0, 0, 0, 1, 1, 1;
  ASSERT_THROW(MinimumSnapReferenceTrajectory(r, tau_vec, path),
               std::length_error);
}

TEST(MinimumSnapReferenceTrajectory, PathRowTooLarge) {
  int r = 4;
  Eigen::VectorXd tau_vec(2);
  tau_vec << 1, 1;
  Eigen::MatrixXd path(4, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3;
  ASSERT_THROW(MinimumSnapReferenceTrajectory(r, tau_vec, path),
               std::length_error);
}

TEST(MinimumSnapReferenceTrajectory, TimeTooSmall) {
  int r = 4;
  Eigen::VectorXd tau_vec(2);
  tau_vec << 1, 1;
  Eigen::MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  ASSERT_THROW(ref.atTime(-1), std::out_of_range);
}

TEST(MinimumSnapReferenceTrajectory, TimeTooLarge) {
  int r = 4;
  Eigen::VectorXd tau_vec(2);
  tau_vec << 1, 1;
  Eigen::MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  ASSERT_NEAR(std::get<0>(ref.atTime(3)).p.x, 2, 1e-7);
  ASSERT_NEAR(std::get<0>(ref.atTime(3)).p.y, 2, 1e-7);
  ASSERT_NEAR(std::get<0>(ref.atTime(3)).p.z, 2, 1e-7);
}

TEST(MinimumSnapReferenceTrajectory, MATLAB) {
  int der_order = 4;
  Eigen::MatrixXd path_matlab = file_utils::load_csv<Eigen::MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) + "/tests/types/data/path_matlab.csv");
  Eigen::MatrixXd tau_vec_matlab = file_utils::load_csv<Eigen::MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) + "/tests/types/data/tau_vec_matlab.csv");
  Eigen::MatrixXd states_matlab = file_utils::load_csv<Eigen::MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) + "/tests/types/data/states_matlab.csv");
  Eigen::MatrixXd t_matlab = file_utils::load_csv<Eigen::MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) + "/tests/types/data/t_matlab.csv");
  auto t0 = std::chrono::high_resolution_clock::now();
  const MinimumSnapReferenceTrajectory ref(der_order, tau_vec_matlab,
                                           path_matlab);
  auto t1 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<float> delta_t = t1 - t0;
  std::cout << "[----------]"
            << " Computation time (10 pts): " << delta_t.count()
            << " seconds\n";

  for (int i = 0; i < t_matlab.rows(); i++) {
    auto ref_t = ref.atTime(t_matlab(i));
    ASSERT_STATES_NEAR(states_matlab.middleRows(5 * i, 5), ref_t);
  }
}

TEST(MinimumSnapReferenceTrajectory, TimeAtStart) {
  int r = 4;
  Eigen::VectorXd tau_vec(2);
  tau_vec << 1, 1;
  Eigen::MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  auto ref_t = ref.atTime(0);
  ASSERT_NEAR(std::get<0>(ref_t).p.x, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t).p.y, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t).p.z, 0, 1e-7);
}

TEST(MinimumSnapReferenceTrajectory, SingleSegment) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 2;
  Eigen::MatrixXd path(2, 3);
  path << 0, 0, 0, 1, 1, 1;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  auto ref_t_st = ref.atTime(0);
  auto ref_t_end = ref.atTime(2);
  ASSERT_NEAR(std::get<0>(ref_t_st).p.x, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t_st).p.y, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t_st).p.z, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t_end).p.x, 1, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t_end).p.y, 1, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t_end).p.z, 1, 1e-7);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
