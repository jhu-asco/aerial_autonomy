#include <aerial_autonomy/types/minimum_snap.h>
#include <gtest/gtest.h>

void ASSERT_VEC_NEAR(const MatrixXd &state1,
                     const std::pair<ParticleState, Snap> &state2,
                     double tol = 1e-7) {
  // ParticleState state2_particle = std::get<0>(state2);
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

TEST(DiscreteReferenceTrajectoryInterpolate, TimeTooSmall) {
  int r = 4;
  VectorXd tau_vec(2);
  tau_vec << 1, 1;
  MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  ASSERT_THROW(ref.atTime(-1), std::out_of_range);
}

TEST(DiscreteReferenceTrajectoryInterpolate, TimeTooLarge) {
  int r = 4;
  VectorXd tau_vec(2);
  tau_vec << 1, 1;
  MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  ASSERT_THROW(ref.atTime(3), std::out_of_range);
}

TEST(MinimumSnapReferenceTrajectory, MATLAB) {
  int r = 4;
  // MatrixXd path_matlab =
  //    load_csv<MatrixXd>("/home/soowon/MATLAB/workspace/path_matlab.csv");

  // MatrixXd path_matlab =
  //    load_csv<MatrixXd>("@DPROJECT_SOURCE_DIR@/tests/types/data/path_matlab.csv");
  MatrixXd path_matlab = load_csv<MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) + "/tests/types/data/path_matlab.csv");

  MatrixXd tau_vec_matlab = load_csv<MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) +
      "/tests/types/data/tau_vec_matlab.csv"); // vec -> Mat...

  MatrixXd states_matlab = load_csv<MatrixXd>(
      std::string(PROJECT_SOURCE_DIR) + "/tests/types/data/states_matlab.csv");
  // std::ifstream file("/home/soowon/MATLAB/workspace/atTimet.csv");
  // std::string tt;
  // std::getline(file, tt);
  // double t_matlab = std::stod(tt);
  MatrixXd t_matlab = load_csv<MatrixXd>(std::string(PROJECT_SOURCE_DIR) +
                                         "/tests/types/data/t_matlab.csv");

  const MinimumSnapReferenceTrajectory ref(r, tau_vec_matlab, path_matlab);
  for (int i = 0; i < t_matlab.rows(); i++) {
    auto ref_t = ref.atTime(t_matlab(i));
    ASSERT_VEC_NEAR(states_matlab.middleRows(5 * i, 5), ref_t);
  }
}

TEST(MinimumSnapReferenceTrajectory, TimeAtStart) {
  int r = 4;
  VectorXd tau_vec(2);
  tau_vec << 1, 1;
  MatrixXd path(3, 3);
  path << 0, 0, 0, 1, 1, 1, 2, 2, 2;
  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);
  auto ref_t = ref.atTime(0);
  ASSERT_NEAR(std::get<0>(ref_t).p.x, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t).p.y, 0, 1e-7);
  ASSERT_NEAR(std::get<0>(ref_t).p.z, 0, 1e-7);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
