#include <aerial_autonomy/types/reference_trajectory.h>
#include <gtest/gtest.h>

TEST(ReferenceTrajectory, TimeTooSmall) {
  ReferenceTrajectory<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  ASSERT_THROW(ref.atTime(-1), std::out_of_range);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
