#include "aerial_autonomy/common/conversions.h"
#include <glog/logging.h>

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

void tfToPositionYaw(PositionYaw &p, const tf::Transform &tf) {
  double yaw = tf::getYaw(tf.getRotation());
  const tf::Vector3 &position = tf.getOrigin();
  p = PositionYaw(position.x(), position.y(), position.z(), yaw);
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
createWaypoint(PositionYaw goal, double desired_joint_angle_1,
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

std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
createWaypoint(PositionYaw goal) {
  std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>> waypoint;
  Eigen::VectorXd goal_control(4);
  goal_control << 1, 0, 0, 0;
  Eigen::VectorXd goal_state(15);
  goal_state << goal.x, goal.y, goal.z, 0, 0, goal.yaw, 0, 0, 0, 0, 0, 0, 0, 0,
      goal.yaw;
  waypoint.reset(
      new Waypoint<Eigen::VectorXd, Eigen::VectorXd>(goal_state, goal_control));
  return waypoint;
}

std::vector<double> vectorEigenToStd(const Eigen::VectorXd &vec_eigen) {
  std::vector<double> vec_std(
      vec_eigen.data(), vec_eigen.data() + vec_eigen.rows() * vec_eigen.cols());
  return vec_std;
}

std::pair<double, double>
accelerationToRollPitch(double yaw, Eigen::Vector3d acceleration_vector,
                        const double &epsilon) {
  double roll, pitch;
  Eigen::Vector3d unit_vec = acceleration_vector.normalized();
  double s_yaw = sin(yaw);
  double c_yaw = cos(yaw);
  double sin_roll = unit_vec[0] * s_yaw - unit_vec[1] * c_yaw;
  if (std::abs(sin_roll) > 1.0) {
    LOG(WARNING) << "sin(roll) > 1";
    sin_roll = std::copysign(1.0, sin_roll);
  }
  roll = std::asin(sin_roll);
  double unit_vec_projection = unit_vec[0] * c_yaw + unit_vec[1] * s_yaw;
  if (std::abs(unit_vec_projection) < epsilon &&
      std::abs(unit_vec[2]) < epsilon) {
    LOG(WARNING) << "Roll is +/-90degrees which is a singularity";
    pitch = 0;
  } else {
    double cos_inv = 1.0 / cos(roll);
    pitch = std::atan2(unit_vec_projection * cos_inv, unit_vec[2] * cos_inv);
  }
  return std::make_pair(roll, pitch);
}

tf::Transform getPose(const parsernode::common::quaddata &data) {
  tf::Transform pose;
  tf::vector3MsgToTF(data.localpos, pose.getOrigin());
  pose.setRotation(tf::createQuaternionFromRPY(data.rpydata.x, data.rpydata.y,
                                               data.rpydata.z));
  return pose;
}

Eigen::Vector3d omegaToRpyDot(const Eigen::Vector3d &omega,
                              const Eigen::Vector3d &rpy,
                              const double max_pitch) {
  Eigen::Matrix3d Mrpy;
  double pitch = rpy[1];
  double abs_pitch = std::abs(pitch);
  if (abs_pitch > max_pitch) {
    LOG(WARNING) << "Pitch close to pi/2 which is a singularity";
    pitch = std::copysign(max_pitch, pitch);
  }
  double s_roll = sin(rpy[0]), c_roll = cos(rpy[0]), t_pitch = tan(pitch);
  double sec_pitch = sqrt(1 + t_pitch * t_pitch);
  Mrpy << 1, s_roll * t_pitch, c_roll * t_pitch, 0, c_roll, -s_roll, 0,
      s_roll * sec_pitch, c_roll * sec_pitch;
  return Mrpy * omega;
}
}
