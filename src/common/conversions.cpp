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

Eigen::Vector3d transformTfToRPY(const tf::Transform &tf) {
  double r, p, y;
  tf.getBasis().getEulerYPR(y, p, r);
  return Eigen::Vector3d(r, p, y);
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
std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
createWayPoint(PositionYaw goal, double desired_joint_angle_1,
               double desired_joint_angle_2) {
  std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>> waypoint;
  Eigen::VectorXd goal_control(6);
  goal_control << 1, 0, 0, 0, desired_joint_angle_1, desired_joint_angle_2;
  Eigen::VectorXd goal_state(21);
  goal_state << goal.x, goal.y, goal.z, 0, 0, goal.yaw, 0, 0, 0, 0, 0, 0, 0, 0,
      goal.yaw, desired_joint_angle_1, desired_joint_angle_2, 0, 0,
      desired_joint_angle_1, desired_joint_angle_2;
  waypoint.reset(
      new Waypoint<Eigen::VectorXd, Eigen::VectorXd>(goal_state, goal_control));
  return waypoint;
}
}
