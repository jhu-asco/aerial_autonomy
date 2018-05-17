#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"

#include <tf_conversions/tf_eigen.h>

using Matrix63d = Eigen::Matrix<double, 6, 3>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

std::tuple<ParticleState, Snap>
QrotorBacksteppingController::getGoalFromReference(
    const ReferenceTrajectory<ParticleState, Snap> &ref) {
  return std::tuple<ParticleState, Snap>(ref.states.at(0), ref.controls.at(0));
}

bool QrotorBacksteppingController::runImplementation(
    QrotorBSState sensor_data, ReferenceTrajectory<ParticleState, Snap> goal,
    QrotorBSControl &control) {
  std::tuple<ParticleState, Snap> current_ref = getGoalFromReference(goal);
  auto desired_state = std::get<0>(current_ref);
  Eigen::Vector3d snap_d = conversions::toEigen(std::get<1>(current_ref));

  // A bunch of conversions and definitions
  Eigen::Matrix3d J =
      Eigen::Vector3d(config_.jx(), config_.jy(), config_.jz()).asDiagonal();
  double m = config_.mass();
  Eigen::Vector3d e(0, 0, 1); // thrust vector in body-frame
  Eigen::Vector3d ag = Eigen::Vector3d(0, 0, -config_.acc_gravity());
  Eigen::Vector3d kp(config_.kp_xy(), config_.kp_xy(), config_.kp_z());
  Eigen::Vector3d kd(config_.kd_xy(), config_.kd_xy(), config_.kd_z());
  Matrix36d K;
  K.leftCols<3>() = kp.asDiagonal();
  K.rightCols<3>() = kd.asDiagonal();

  Matrix6d A = Eigen::MatrixXd::Zero(6, 6);
  A.topRightCorner<3, 3>() = Eigen::MatrixXd::Identity(3, 3);

  Matrix63d B = Eigen::MatrixXd::Zero(6, 3);
  B.bottomLeftCorner<3, 3>() = (1. / m) * Eigen::MatrixXd::Identity(3, 3);

  Eigen::Matrix3d R;
  tf::matrixTFToEigen(sensor_data.pose.getBasis(), R);
  double thrust = sensor_data.thrust;
  double thrust_dot = sensor_data.thrust_dot;

  Eigen::Vector3d p;
  tf::vectorTFToEigen(sensor_data.pose.getOrigin(), p);
  Eigen::Vector3d p_d = conversions::toEigen(desired_state.p);
  Eigen::Vector3d v;
  tf::vectorTFToEigen(sensor_data.v, v);
  Eigen::Vector3d v_d = conversions::toEigen(desired_state.v);
  Eigen::Vector3d acc_d = conversions::toEigen(desired_state.a);
  Eigen::Vector3d jerk_d = conversions::toEigen(desired_state.j);
  Eigen::Vector3d w;
  tf::vectorTFToEigen(sensor_data.w, w);

  Eigen::Vector3d f = m * ag;
  Eigen::Vector3d g = R * e * thrust;

  Vector6d x, x_dot;
  x.head<3>() = p;
  x.tail<3>() = v;
  x_dot = A * x + B * (f + g);

  Vector6d x_d, x_d_dot, x_d_ddot;
  x_d.head<3>() = p_d;
  x_d.tail<3>() = v_d;
  x_d_dot.head<3>() = v_d;
  x_d_dot.tail<3>() = acc_d;
  x_d_ddot.head<3>() = acc_d;
  x_d_ddot.tail<3>() = jerk_d;

  Eigen::Matrix3d w_hat;
  tf::Matrix3x3 w_hat_tf = math::hat(sensor_data.w);
  tf::matrixTFToEigen(w_hat_tf, w_hat);

  // The actual controller computations
  auto z0 = x - x_d;
  auto z0_dot = x_dot - x_d_dot;

  Eigen::Vector3d g_d = m * acc_d - K * z0 - f;
  Eigen::Vector3d z1 = g - g_d;
  Eigen::Vector3d g_d_dot = m * jerk_d - K * z0_dot;
  Eigen::Vector3d g_dot = R * (w_hat * e * thrust + e * thrust_dot);
  auto z1_dot = g_dot - g_d_dot;

  auto x_ddot = A * x_dot + B * g_dot;
  auto g_d_ddot = m * snap_d - K * (x_ddot - x_d_ddot);

  Eigen::Vector3d a_d = g_d_dot - B.transpose() * P_ * z0 - config_.k1() * z1;
  Eigen::Vector3d a_d_dot =
      g_d_ddot - B.transpose() * P_ * z0_dot - config_.k1() * z1_dot;
  auto z2 = g_dot - a_d;

  Eigen::Vector3d b_d = a_d_dot - z1 - config_.k2() * z2;

  auto snap_cmd = R.transpose() * b_d - thrust * w_hat * w_hat * e -
                  2.0 * thrust_dot * w_hat * e;

  if (fabs(thrust) > config_.thrust_eps()) {
    tf::vectorEigenToTF(J * (e.cross(snap_cmd) / thrust) - (J * w).cross(w),
                        control.torque);
  } else {
    control.torque = tf::Vector3(0, 0, 0);
  }
  control.thrust_ddot = e.dot(snap_cmd);
  return true;
}

ControllerStatus QrotorBacksteppingController::isConvergedImplementation(
    QrotorBSState sensor_data, ReferenceTrajectory<ParticleState, Snap> goal) {
  ControllerStatus controller_status(ControllerStatus::Completed);
  return controller_status;
}
