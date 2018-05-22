#include "aerial_autonomy/common/math.h"

#include <gtest/gtest.h>

TEST(AngleWrapTests, Zero) { ASSERT_NEAR(0, math::angleWrap(0), 1e-10); }

TEST(AngleWrapTests, PositiveNum) {
  ASSERT_NEAR(.1, math::angleWrap(.1), 1e-10);
}

TEST(AngleWrapTests, NegativeNum) {
  ASSERT_NEAR(-.1, math::angleWrap(-.1), 1e-10);
}

TEST(AngleWrapTests, OnePositiveToWrap) {
  ASSERT_NEAR(-M_PI / 2, math::angleWrap(3 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, OnePositiveToPositiveWrap) {
  ASSERT_NEAR(M_PI / 2, math::angleWrap(5 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, OneNegativeToPositiveWrap) {
  ASSERT_NEAR(M_PI / 2, math::angleWrap(-3 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, OneNegativeToNegativeWrap) {
  ASSERT_NEAR(-M_PI / 2, math::angleWrap(-5 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, TwoPositiveToWrap) {
  ASSERT_NEAR(-M_PI / 2, math::angleWrap(7 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, TwoPositiveToPositiveWrap) {
  ASSERT_NEAR(M_PI / 2, math::angleWrap(9 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, TwoNegativeToPositiveWrap) {
  ASSERT_NEAR(M_PI / 2, math::angleWrap(-7 * M_PI / 2), 1e-10);
}

TEST(AngleWrapTests, TwoNegativeToNegativeWrap) {
  ASSERT_NEAR(-M_PI / 2, math::angleWrap(-9 * M_PI / 2), 1e-10);
}

///

TEST(ClampTests, InBounds) {
  ASSERT_EQ(math::clamp(0, -1, 1), 0);
  ASSERT_EQ(math::clamp(0.5, -1, 1), 0.5);
  ASSERT_EQ(math::clamp(-0.5, -1, 1), -0.5);
}

TEST(ClampTests, Max) {
  ASSERT_EQ(math::clamp(2, -1, 1), 1);
  ASSERT_EQ(math::clamp(1, -1, 1), 1);
}

TEST(ClampTests, Min) {
  ASSERT_EQ(math::clamp(-2, -2, 1), -2);
  ASSERT_EQ(math::clamp(-100, -2, 1), -2);
}

TEST(MapTests, InBounds) { ASSERT_EQ(math::map(5, -10, 10, -1, 1), 0.5); }

TEST(MapTests, InBoundsNeg) { ASSERT_EQ(math::map(-5, -10, 10, -1, 1), -0.5); }

TEST(MapTests, OutOfBoundsMax) { ASSERT_EQ(math::map(15, -10, 10, -1, 1), 1); }

TEST(MapTests, OutOfBoundsMin) {
  ASSERT_EQ(math::map(-15, -10, 10, -1, 1), -1);
}

TEST(HatTests, HatIsCrossProduct) {
  Eigen::Vector3d v(3, -1, 2);
  Eigen::Matrix3d v_hat = math::hat(v); 
  ASSERT_NEAR(v.dot(v_hat * v), 0, 1e-6);

  Eigen::Vector3d v2(-5, -4, -6);
  Eigen::Matrix3d v2_hat = math::hat(v2); 
  ASSERT_NEAR(v2.dot(v2_hat * v2), 0, 1e-6);

  Eigen::Vector3d v_hat_v2 =  v_hat * v2;
  Eigen::Vector3d v_cross_v2 = v.cross(v2);

  ASSERT_NEAR(v_hat_v2(0), v_cross_v2(0), 1e-6);
  ASSERT_NEAR(v_hat_v2(1), v_cross_v2(1), 1e-6);
  ASSERT_NEAR(v_hat_v2(2), v_cross_v2(2), 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
