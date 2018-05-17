#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"

#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>

bool QrotorBacksteppingController::runImplementation(
    QrotorBSState sensor_data, ReferenceTrajectory<ParticleState, Snap> goal,
    QrotorBSControl &control) {
  std::tuple<ParticleState, Snap> desired_state = getGoalFromReference(goal);

  // A bunch of conversions and definitions
  Eigen::Matrix3d J = config_.J();
  double m = config_.mass();
  Eigen::Vector3d e(0, 0, 1); // thrust vector in body-frame
  Eigen::Vector3d ag = config_.acc_gravity();
  Eigen::Vector3d kp(config_.kp_xy(), config_.kp_xy(), config_.kp_z());
  Eigen::Vector3d kd(config_.kd_xy(), config_.kd_xy(), config_.kd_z());
  Eigen::MatrixXd<6, 3> K;
  K.topRows<3>() = kp.asDiagonal();
  K.bottomRows<3>() = kd.asDiagonal();

  Eigen::Matrix6d A = Eigen::MatrixXd::Zero(6, 6);
  A.topRightCorner<3, 3> = Eigen::MatrixXs::Ones(3, 3);

  Eigen::MatrixXd<6, 3> B = Eigen::MatrixXd::Zero(6, 3);
  B.bottomLeftCorner<3, 3> = (1. / m) * Eigen::MatrixXs::Ones(3, 3);

  Eigen::Matrix3d R;
  tf::matrixTFToEigen(sensor_data.pose.getBasis(), R);
  double thrust = sensor_data.thrust;
  double thrust_dot = sensor_data.thrust_dot;

  Eigen::Vector3d p;
  tf::vectorTFToEigen(sensor_data.pose.getOrigin(), p);
  Eigen::Vector3d p_d = conversions::positionToEigen(desired_state.p);
  Eigen::Vector3d v;
  tf::vectorTFToEigen(sensor_data.v, v);
  Eigen::Vector3d v_d = conversions::velocityToEigen(desired_state.v);
  Eigen::Vector3d acc_d = conversions::accelerationToEigen(desired_state.a);
  Eigen::Vector3d jerk_d = conversions::jerkToEigen(desired_state.j);
  Eigen::Vector3d snap_d;

  Eigen::Vector3d f = m * ag;
  Eigen::Vector3d g = R * e * thrust;

  Eigen::Vector6d x, x_dot;
  x << p.transpose() << v.transpose();
  x_dot = A * x + B * (f + g);

  Eigen::Vector6d x_d, x_d_dot, x_d_ddot;
  x_d << p_d.transpose() << v_d.transpose();
  x_d_dot << v_d.transpose() << acc_d.transpose();
  x_d_ddot << acc_d.transpose() << jerk_d.transpose();

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

  Eigen::Vector3d a_d = g_d_dot - B.transpose() * P * z0 - config_.k1() * z1;
  Eigen::Vector3d a_d_dot =
      g_d_ddot - B.transpose() * P * z0_dot - config_.k1() * z1_dot;
  auto z2 = g_dot - a_d;

  Eigen::Vector3d b_d = a_d_dot - z1 - config_.k2() * z2;

  tf::Matrix3x3 w_hat_tf = math::hat(sensor_data.w);
  Eigen::Matrix3d w_hat;
  tf::matrixTFToEigen(w_hat_tf, w_hat);

  auto snap_d = R.transpose() * b_d - thrust * w_hat * w_hat * e -
                2.0 * thrust_dot * w_hat;

  if (fabs(thrust) > config_.thrust_eps) {
    tf::vectorEigenToTF(J * (e.cross(snap_d) / thrust) - (J * w).cross(w),
                        control.torque);
  } else {
    control.torque = tf::Vector3(0, 0, 0);
  }
  control.thrust_ddot = e.dot(snap_d);
}
ControllerStatus QrotorBacksteppingController::isConvergedImplementation(
    QrotorBSState sensor_data, ReferenceTrajectory<ParticleState, Snap> goal) {}
