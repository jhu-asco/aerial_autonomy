#include <gtest/gtest.h>

#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/tests/test_utils.h"

using namespace conversions;
using namespace test_utils;

TEST(PositionYawToTf, Zero) {
  PositionYaw p(0, 0, 0, 0);
  tf::Transform p_tf;
  positionYawToTf(p, p_tf);
  ASSERT_TF_NEAR(
      p_tf, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)));
}

TEST(PositionYawToTf, ZeroYaw) {
  PositionYaw p(1, 2, 3, 0);
  tf::Transform p_tf;
  positionYawToTf(p, p_tf);
  ASSERT_TF_NEAR(
      p_tf, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 3)));
}

TEST(PositionYawToTf, PositiveYaw) {
  PositionYaw p(-1, 2, 50, 0.1);
  tf::Transform p_tf;
  positionYawToTf(p, p_tf);
  ASSERT_TF_NEAR(p_tf, tf::Transform(tf::createQuaternionFromRPY(0, 0, 0.1),
                                     tf::Vector3(-1, 2, 50)));
}

TEST(PositionYawToTf, NegativeYaw) {
  PositionYaw p(-1, 2, 50, -0.1);
  tf::Transform p_tf;
  positionYawToTf(p, p_tf);
  ASSERT_TF_NEAR(p_tf, tf::Transform(tf::createQuaternionFromRPY(0, 0, -0.1),
                                     tf::Vector3(-1, 2, 50)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
