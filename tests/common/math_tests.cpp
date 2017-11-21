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

<<<<<<< HEAD
TEST(TransformTests, StdVector) {
  std::vector<double> input = {1.0, 2.0, 3.0, M_PI / 2, -M_PI / 3, M_PI / 4};
  tf::Transform out = math::getTransformFromVector(input);
  // Check origin
  tf::Vector3 origin = out.getOrigin();
  ASSERT_EQ(origin.getX(), input[0]);
  ASSERT_EQ(origin.getY(), input[1]);
  ASSERT_EQ(origin.getZ(), input[2]);
  // Check rpy
  tf::Matrix3x3 input_basis;
  input_basis.setEulerYPR(input[5], input[4], input[3]);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ASSERT_NEAR(input_basis[i][j], out.getBasis()[i][j], 1e-4);
    }
  }
}

TEST(TransformTests, StdVectorThrow) {
  std::vector<double> input = {1.0, 2.0, 3.0, 0.0, 0.0};
  ASSERT_THROW(math::getTransformFromVector(input), std::runtime_error);
}

TEST(TransformsTests, StdVector) {
  std::vector<double> input = {1.0, 2.0, 3.0, M_PI / 2, -M_PI / 3, M_PI / 4,
                               0,   -1,  -2,  0,        M_PI,      0};
  std::vector<tf::Transform> tfs = math::getTransformsFromVector(input);

  for (unsigned int i = 0; i < tfs.size(); i++) {
    tf::Transform out = tfs[i];
    // Check origin
    tf::Vector3 origin = out.getOrigin();
    ASSERT_EQ(origin.getX(), input[6 * i + 0]);
    ASSERT_EQ(origin.getY(), input[6 * i + 1]);
    ASSERT_EQ(origin.getZ(), input[6 * i + 2]);
    // Check rpy
    tf::Matrix3x3 input_basis;
    input_basis.setEulerYPR(input[6 * i + 5], input[6 * i + 4],
                            input[6 * i + 3]);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        ASSERT_NEAR(input_basis[i][j], out.getBasis()[i][j], 1e-4);
      }
    }
  }
}

TEST(TransformsTests, StdVectorThrow) {
  std::vector<double> input = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_THROW(math::getTransformsFromVector(input), std::runtime_error);
}

///
TEST(MapTests, InBounds) { ASSERT_EQ(math::map(5, -10, 10, -1, 1), 0.5); }

TEST(MapTests, InBoundsNeg) { ASSERT_EQ(math::map(-5, -10, 10, -1, 1), -0.5); }

TEST(MapTests, OutOfBoundsMax) { ASSERT_EQ(math::map(15, -10, 10, -1, 1), 1); }

TEST(MapTests, OutOfBoundsMin) {
  ASSERT_EQ(math::map(-15, -10, 10, -1, 1), -1);
}

=======
>>>>>>> 37cb0139ced4a79f59068b1dbc2c6839f6f9c3ec
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
