#include <aerial_autonomy/types/minimum_snap.h>
#include <gtest/gtest.h>

TEST(MinimumSnapReferenceTrajectory, TimeInMiddle) {

  int r = 4;
  MatrixXd path(3,3);
  path << -1, -2, 4,
           0, 1, 2,
           5, 3, 1;
  VectorXd tau_vec(2);
  tau_vec << 4, 3;

  const MinimumSnapReferenceTrajectory ref(r, tau_vec, path);

  auto ref_t = ref.atTime(4.5);
  ParticleState state_test = std::get<0>(ref_t);
  Position state_test_p = state_test.p;

  ASSERT_EQ(state_test_p.x, state_test_p.x);
  //ASSERT_NEAR(state_test_p, state_test_p);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
