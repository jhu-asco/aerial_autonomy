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

void compareProtoToTf(double x, double y, double z, double roll, double pitch,
                      double yaw) {
  config::Transform ptf;
  ptf.mutable_position()->set_x(x);
  ptf.mutable_position()->set_y(y);
  ptf.mutable_position()->set_z(z);
  ptf.mutable_rotation()->set_r(roll);
  ptf.mutable_rotation()->set_p(pitch);
  ptf.mutable_rotation()->set_y(yaw);

  tf::Transform tf = conversions::protoTransformToTf(ptf);
  ASSERT_TF_NEAR(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
                               tf::Vector3(x, y, z)),
                 tf);
}

TEST(ProtoTransformToTfTransform, Zero) { compareProtoToTf(0, 0, 0, 0, 0, 0); }

TEST(ProtoTransformToTfTransform, NonZero) {
  compareProtoToTf(-3.2, 1, 5, 0.1, 0.5, -1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
