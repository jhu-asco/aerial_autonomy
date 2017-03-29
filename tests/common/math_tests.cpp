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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
