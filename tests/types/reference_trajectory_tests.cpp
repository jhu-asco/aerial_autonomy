#include <aerial_autonomy/types/discrete_reference_trajectory_closest.h>
#include <aerial_autonomy/types/discrete_reference_trajectory_interpolate.h>
#include <aerial_autonomy/types/waypoint.h>
#include <gtest/gtest.h>

TEST(DiscreteReferenceTrajectoryInterpolate, TimeTooSmall) {
  DiscreteReferenceTrajectoryInterpolate<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  ASSERT_THROW(ref.atTime(-1), std::out_of_range);
}

TEST(DiscreteReferenceTrajectoryInterpolate, TimeTooLarge) {
  DiscreteReferenceTrajectoryInterpolate<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  ASSERT_THROW(ref.atTime(5.1), std::out_of_range);
}

TEST(DiscreteReferenceTrajectoryInterpolate, TimeInMiddle) {
  DiscreteReferenceTrajectoryInterpolate<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};

  auto ref_t = ref.atTime(4.5);
  ASSERT_NEAR(std::get<0>(ref_t), 4.5, 1e-6);
  ASSERT_NEAR(std::get<1>(ref_t), 4.5, 1e-6);

  ref_t = ref.atTime(3.1);
  ASSERT_NEAR(std::get<0>(ref_t), 3.1, 1e-6);
  ASSERT_NEAR(std::get<1>(ref_t), 3.1, 1e-6);
}
TEST(DiscreteReferenceTrajectoryInterpolate, TimeDiffTooSmall) {
  DiscreteReferenceTrajectoryInterpolate<double, double> ref;
  ref.ts = {0, 1, 1+1e-8, 2, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};
  ASSERT_THROW(ref.atTime(1+1e-9), std::logic_error);
}

TEST(DiscreteReferenceTrajectoryClosest, ClosestBefore) {
  DiscreteReferenceTrajectoryClosest<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};
  auto ref_t = ref.atTime(4.1);
  ASSERT_EQ(std::get<0>(ref_t), 4);
  ASSERT_EQ(std::get<1>(ref_t), 4);
}

TEST(DiscreteReferenceTrajectoryClosest, ClosestAfter) {
  DiscreteReferenceTrajectoryClosest<double, double> ref;
  ref.ts = {0, 1, 2, 3, 4, 5};
  ref.states = {0, 1, 2, 3, 4, 5};
  ref.controls = {0, 1, 2, 3, 4, 5};
  auto ref_t = ref.atTime(3.9);
  ASSERT_EQ(std::get<0>(ref_t), 4);
  ASSERT_EQ(std::get<1>(ref_t), 4);
}

TEST(Waypoint, Get) {
  Waypoint<double, double> ref(1, 2);
  auto ref_t = ref.atTime(0);
  ASSERT_EQ(std::get<0>(ref_t), 1);
  ASSERT_EQ(std::get<1>(ref_t), 2);

  ref_t = ref.atTime(12);
  ASSERT_EQ(std::get<0>(ref_t), 1);
  ASSERT_EQ(std::get<1>(ref_t), 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
