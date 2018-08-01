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

void compareEigenToArma(const Eigen::MatrixXd &m_eig, const arma::mat &m_arma) {
  ASSERT_EQ(m_eig.rows(), m_arma.n_rows);
  ASSERT_EQ(m_eig.cols(), m_arma.n_cols);
  for (int i = 0; i < m_eig.rows(); i++) {
    for (int j = 0; j < m_eig.cols(); j++) {
      ASSERT_NEAR(m_arma(i, j), m_eig(i, j), 1e-5);
    }
  }
}

void compareArma(const arma::mat &m1, const arma::mat &m2) {
  ASSERT_EQ(m1.n_rows, m2.n_rows);
  ASSERT_EQ(m1.n_cols, m2.n_cols);
  for (unsigned int i = 0; i < m1.n_rows; i++) {
    for (unsigned int j = 0; j < m1.n_cols; j++) {
      ASSERT_NEAR(m1(i, j), m2(i, j), 1e-5);
    }
  }
}

TEST(EigenToArma, SquareMat) {
  Eigen::Matrix3d m = Eigen::MatrixXd::Zero(3, 3);
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 3;

  arma::mat m_arma = conversions::eigenToArma(m);
  compareEigenToArma(m, m_arma);
}
TEST(EigenToArma, FatMat) {
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(3, 5);
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 0;
  m(1, 4) = -2;

  arma::mat m_arma = conversions::eigenToArma(m);
  compareEigenToArma(m, m_arma);
}

TEST(EigenToArma, SkinnyMat) {
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(5, 3);
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 3;
  m(4, 0) = 2;
  m(4, 1) = -2;

  arma::mat m_arma = conversions::eigenToArma(m);
  compareEigenToArma(m, m_arma);
}

TEST(EigenToArma, ConvertBack) {
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(5, 3);
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 3;
  m(4, 0) = 2;
  m(4, 1) = -2;

  Eigen::MatrixXd m2 = conversions::armaToEigen(conversions::eigenToArma(m));
  ASSERT_TRUE(m.isApprox(m2));
}

TEST(ArmaToEigen, SquareMat) {
  arma::mat m(3, 3);
  m.zeros();
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 1;

  Eigen::MatrixXd m_eig = conversions::armaToEigen(m);
  compareEigenToArma(m_eig, m);
}

TEST(ArmaToEigen, FatMat) {
  arma::mat m(3, 5);
  m.zeros();
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 1;
  m(2, 4) = 5;

  Eigen::MatrixXd m_eig = conversions::armaToEigen(m);
  compareEigenToArma(m_eig, m);
}

TEST(ArmaToEigen, SkinnyMat) {
  arma::mat m(6, 3);
  m.zeros();
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 1;
  m(4, 1) = 5;
  m(5, 0) = -3;

  Eigen::MatrixXd m_eig = conversions::armaToEigen(m);
  compareEigenToArma(m_eig, m);
}

TEST(ArmaToEigen, ConvertBack) {
  arma::mat m(6, 3);
  m.zeros();
  m(1, 0) = 5;
  m(0, 2) = -1;
  m(2, 2) = 1;
  m(4, 1) = 5;
  m(5, 0) = -3;
  arma::mat m2 = conversions::eigenToArma(conversions::armaToEigen(m));
  compareArma(m, m2);
}

TEST(EigenToStdVec, VectorXdToStdVecDouble) {
  Eigen::VectorXd vec_eigen(5);
  vec_eigen << 1, 2, 3, 4, 5;
  std::vector<double> vec_std = conversions::vectorEigenToStd(vec_eigen);
  for (int i = 0; i < vec_eigen.size(); i++) {
    ASSERT_NEAR(vec_eigen(i), vec_std[i], 1e-7);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
