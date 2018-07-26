#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <gtest/gtest.h>

using namespace test_utils;

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
  Eigen::Vector3d v_hat_v = v_hat * v;
  ASSERT_VEC_NEAR(v_hat_v, Eigen::Vector3d(0, 0, 0), 1e-6);

  Eigen::Vector3d v2(-5, -4, -6);
  Eigen::Matrix3d v2_hat = math::hat(v2);
  Eigen::Vector3d v2_hat_v2 = v2_hat * v2;
  ASSERT_VEC_NEAR(v2_hat_v2, Eigen::Vector3d(0, 0, 0), 1e-6);

  Eigen::Vector3d v_hat_v2 = v_hat * v2;
  Eigen::Vector3d m_v2_hat_v = -v2_hat * v;
  Eigen::Vector3d v_cross_v2 = v.cross(v2);
  ASSERT_VEC_NEAR(v_hat_v2, v_cross_v2, 1e-6);
  ASSERT_VEC_NEAR(v_hat_v2, m_v2_hat_v, 1e-6);
}

TEST(CumsumTests, CumulativeSumForEigen) {
  Eigen::VectorXd vec_eigen(5);
  vec_eigen << 1, 2, 3, 4, 5;
  Eigen::VectorXd vec_cumsum_eigen(6);
  vec_cumsum_eigen << 0, 1, 3, 6, 10, 15;
  Eigen::VectorXd vec_cumsum_eigen_result = math::cumsumEigen(vec_eigen);
  for (int i = 0; i < vec_cumsum_eigen.size(); i++) {
    ASSERT_NEAR(vec_cumsum_eigen(i), vec_cumsum_eigen_result(i), 1e-7);
  }
}

TEST(CumprodTests, CumprodOfExponent) {
  double tau = 0.1;
  int size = 5;
  std::vector<double> tau_exp = math::cumulativeExp(tau, size);
  for (int i = 0; i < static_cast<int>(tau_exp.size()); i++) {
    ASSERT_NEAR(tau_exp[i], pow(tau, i), 1e-7);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
