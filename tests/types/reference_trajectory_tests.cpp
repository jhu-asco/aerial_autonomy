#include <aerial_autonomy/types/reference_trajectory.h>
#include <gtest/gtest.h>

TEST(ReferenceTrajectory, TimeTooSmall) {
  ReferenceTrajectory<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  ASSERT_THROW(ref.atTime(-1), std::out_of_range);
}

TEST(ReferenceTrajectory, TimeTooLarge) {
  ReferenceTrajectory<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  ASSERT_THROW(ref.atTime(5.1), std::out_of_range);
}

TEST(ReferenceTrajectory, TimeInMiddle) {
  ReferenceTrajectory<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  auto reft = ref.atTime(4.5);
  ASSERT_NEAR(std::get<0>(reft), 4.5, 1e-6);
  ASSERT_NEAR(std::get<1>(reft), 4.5, 1e-6);

  reft = ref.atTime(3.1);
  ASSERT_NEAR(std::get<0>(reft), 3.1, 1e-6);
  ASSERT_NEAR(std::get<1>(reft), 3.1, 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
