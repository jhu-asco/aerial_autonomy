#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"

#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>

std::pair<ParticleState, Snap>
QrotorBacksteppingController::getGoalFromReference(
    double t, const ReferenceTrajectory<ParticleState, Snap> &ref) {
  return ref.atTime(t);
}

bool QrotorBacksteppingController::runImplementation(
    std::pair<double, QrotorBacksteppingState> sensor_data,
    std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
    QrotorBacksteppingControl &control) {
  QrotorBacksteppingState current_state = std::get<1>(sensor_data);
  double current_time = std::get<0>(sensor_data);

  std::pair<ParticleState, Snap> current_ref;
  try {
    current_ref = getGoalFromReference(current_time, *goal);
  } catch (std::logic_error e) {
    LOG(WARNING) << e.what() << std::endl;
    return false;
  }

  ParticleState desired_state = std::get<0>(current_ref);
  Eigen::Vector3d snap_d = conversions::toEigen(std::get<1>(current_ref));

  // A bunch of conversions and definitions
  Eigen::Matrix3d R;
  tf::matrixTFToEigen(current_state.pose.getBasis(), R);
  double thrust = current_state.thrust;
  double thrust_dot = current_state.thrust_dot;

  Eigen::Vector3d p;
  tf::vectorTFToEigen(current_state.pose.getOrigin(), p);
  Eigen::Vector3d p_d = conversions::toEigen(desired_state.p);
  Eigen::Vector3d v;
  tf::vectorTFToEigen(current_state.v, v);
  Eigen::Vector3d v_d = conversions::toEigen(desired_state.v);
  Eigen::Vector3d acc_d = conversions::toEigen(desired_state.a);
  Eigen::Vector3d jerk_d = conversions::toEigen(desired_state.j);
  Eigen::Vector3d w;
  tf::vectorTFToEigen(current_state.w, w);

  Eigen::Vector3d f = m_ * ag_;
  Eigen::Vector3d g = R * e_ * thrust;

  Vector6d x, x_dot;
  x.head<3>() = p;
  x.tail<3>() = v;
  x_dot = A_ * x + B_ * (f + g);

  Vector6d x_d, x_d_dot, x_d_ddot;
  x_d.head<3>() = p_d;
  x_d.tail<3>() = v_d;
  x_d_dot.head<3>() = v_d;
  x_d_dot.tail<3>() = acc_d;
  x_d_ddot.head<3>() = acc_d;
  x_d_ddot.tail<3>() = jerk_d;

  Eigen::Vector3d w_eig;
  tf::vectorTFToEigen(current_state.w, w_eig);
  Eigen::Matrix3d w_hat = math::hat(w_eig);

  // The actual controller computations
  auto z0 = x - x_d;
  auto z0_dot = x_dot - x_d_dot;

  Eigen::Vector3d g_d = m_ * acc_d - K_ * z0 - f;
  Eigen::Vector3d z1 = g - g_d;
  Eigen::Vector3d g_d_dot = m_ * jerk_d - K_ * z0_dot;
  Eigen::Vector3d g_dot = R * (w_hat * e_ * thrust + e_ * thrust_dot);
  auto z1_dot = g_dot - g_d_dot;

  auto x_ddot = A_ * x_dot + B_ * g_dot;
  auto g_d_ddot = m_ * snap_d - K_ * (x_ddot - x_d_ddot);

  Eigen::Vector3d a_d = g_d_dot - B_.transpose() * P_ * z0 - config_.k1() * z1;
  Eigen::Vector3d a_d_dot =
      g_d_ddot - B_.transpose() * P_ * z0_dot - config_.k1() * z1_dot;
  auto z2 = g_dot - a_d;

  Eigen::Vector3d b_d = a_d_dot - z1 - config_.k2() * z2;

  auto snap_cmd = R.transpose() * b_d - thrust * w_hat * w_hat * e_ -
                  2.0 * thrust_dot * w_hat * e_;

  if (fabs(thrust) > config_.thrust_eps()) {
    tf::vectorEigenToTF(J_ * (e_.cross(snap_cmd) / thrust) - (J_ * w).cross(w),
                        control.torque);
  } else {
    control.torque = tf::Vector3(0, 0, 0);
  }
  control.thrust_ddot = e_.dot(snap_cmd);
  DATA_LOG("qrotor_backstepping_controller")
      << p(0) << p(1) << p(2) << p_d(0) << p_d(1) << p_d(2) << v(0) << v(1)
      << v(2) << v_d(0) << v_d(1) << v_d(2) << DataStream::endl;
  return true;
}

ControllerStatus QrotorBacksteppingController::isConvergedImplementation(
    std::pair<double, QrotorBacksteppingState> sensor_data,
    std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal) {
  ControllerStatus controller_status = ControllerStatus::Active;
  QrotorBacksteppingState current_state = std::get<1>(sensor_data);
  ParticleState end_goal = goal->goal(sensor_data.first);

  const config::Velocity tolerance_vel = config_.goal_velocity_tolerance();
  const config::Position tolerance_pos = config_.goal_position_tolerance();

  Velocity current_velocity(current_state.v.x(), current_state.v.y(),
                            current_state.v.z());
  tf::Vector3 current_pos_tf = current_state.pose.getOrigin();
  Position current_position(current_pos_tf.x(), current_pos_tf.y(),
                            current_pos_tf.z());

  Velocity velocity_diff = end_goal.v - current_velocity;
  Position position_diff = end_goal.p - current_position;
  controller_status << "Error pos, vel: " << position_diff.x << position_diff.y
                    << position_diff.z << velocity_diff.x << velocity_diff.y
                    << velocity_diff.z;
  // TODO: use common vector3 interface to compare two vectors
  if (std::abs(velocity_diff.x) < tolerance_vel.vx() &&
      std::abs(velocity_diff.y) < tolerance_vel.vy() &&
      std::abs(velocity_diff.z) < tolerance_vel.vz() &&
      std::abs(position_diff.x) < tolerance_pos.x() &&
      std::abs(position_diff.y) < tolerance_pos.y() &&
      std::abs(position_diff.z) < tolerance_pos.z()) {
    controller_status.setStatus(ControllerStatus::Completed);
  }
  return controller_status;
}
