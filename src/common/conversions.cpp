#include "aerial_autonomy/common/conversions.h"

#include <tf_conversions/tf_eigen.h>

/**
 * @brief Namespace for converting eign to tf matrices
 */
namespace conversions {
// \todo Matt add tests for transformMatrix4dToTf and transformRPYToTf
void transformMatrix4dToTf(const Eigen::Matrix4d &e, tf::Transform &tf) {
  Eigen::Affine3d e_affine;
  e_affine.matrix() = e;
  tf::transformEigenToTF(e_affine, tf);
}

void transformRPYToTf(double r, double p, double y, tf::Transform &tf) {
  tf.setIdentity();
  tf.setRotation(tf::createQuaternionFromRPY(r, p, y));
}

tf::Transform protoTransformToTf(const config::Transform &tf) {
  return tf::Transform(
      tf::createQuaternionFromRPY(tf.rotation().r(), tf.rotation().p(),
                                  tf.rotation().y()),
      tf::Vector3(tf.position().x(), tf.position().y(), tf.position().z()));
}

void positionYawToTf(const PositionYaw &p, tf::Transform &tf) {
  tf.setRotation(tf::createQuaternionFromRPY(0, 0, p.yaw));
  tf.setOrigin(tf::Vector3(p.x, p.y, p.z));
}

PositionYaw protoPositionYawToPositionYaw(config::PositionYaw p) {
  return PositionYaw(p.position().x(), p.position().y(), p.position().z(),
                     p.yaw());
}

arma::mat eigenToArma(const Eigen::MatrixXd &m) {
  return arma::mat(m.data(), m.rows(), m.cols());
}

Eigen::MatrixXd armaToEigen(const arma::mat &m) {
  Eigen::MatrixXd m_eig(m.n_rows, m.n_cols);
  std::memcpy(m_eig.data(), m.memptr(), sizeof(double) * m.n_rows * m.n_cols);
  return m_eig;
}

std::vector<double> vectorEigenToStd(const Eigen::VectorXd &vec_eigen) {
  std::vector<double> vec_std(
      vec_eigen.data(), vec_eigen.data() + vec_eigen.rows() * vec_eigen.cols());
  return vec_std;
}
}
