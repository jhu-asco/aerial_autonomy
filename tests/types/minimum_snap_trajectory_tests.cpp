#include <aerial_autonomy/types/minimum_snap.h>
#include <gtest/gtest.h>


void ASSERT_VEC_NEAR(const MatrixXd &state1, const std::pair<ParticleState, Snap> &state2, double tol = 1e-5) {
  //ParticleState state2_particle = std::get<0>(state2);
  ASSERT_NEAR(state1(0,0), std::get<0>(state2).p.x, tol);
  ASSERT_NEAR(state1(0,1), std::get<0>(state2).p.y, tol);
  ASSERT_NEAR(state1(0,2), std::get<0>(state2).p.z, tol);

  ASSERT_NEAR(state1(1,0), std::get<0>(state2).v.x, tol);
  ASSERT_NEAR(state1(1,1), std::get<0>(state2).v.y, tol);
  ASSERT_NEAR(state1(1,2), std::get<0>(state2).v.z, tol);

  ASSERT_NEAR(state1(2,0), std::get<0>(state2).a.x, tol);
  ASSERT_NEAR(state1(2,1), std::get<0>(state2).a.y, tol);
  ASSERT_NEAR(state1(2,2), std::get<0>(state2).a.z, tol);

  ASSERT_NEAR(state1(3,0), std::get<0>(state2).j.x, tol);
  ASSERT_NEAR(state1(3,1), std::get<0>(state2).j.y, tol);
  ASSERT_NEAR(state1(3,2), std::get<0>(state2).j.z, tol);

  ASSERT_NEAR(state1(4,0), std::get<1>(state2).x, tol);
  ASSERT_NEAR(state1(4,1), std::get<1>(state2).y, tol);
  ASSERT_NEAR(state1(4,2), std::get<1>(state2).z, tol);
}

TEST(MinimumSnapReferenceTrajectory, TimeInMiddle) {
  int r = 4;

  MatrixXd path_matlab = load_csv<MatrixXd>("/home/soowon/MATLAB/workspace/path.csv");
  MatrixXd tau_vec_matlab = load_csv<MatrixXd>("/home/soowon/MATLAB/workspace/tau_vec.csv");
  MatrixXd states_matlab = load_csv<MatrixXd>("/home/soowon/MATLAB/workspace/states.csv");
  std::ifstream file("/home/soowon/MATLAB/workspace/atTimet.csv");
  std::string tt;
  std::getline(file, tt);
  double t_matlab = std::stod(tt);

  const MinimumSnapReferenceTrajectory ref(r, tau_vec_matlab, path_matlab);
  auto ref_t = ref.atTime(t_matlab);

  ASSERT_VEC_NEAR(states_matlab, ref_t);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
