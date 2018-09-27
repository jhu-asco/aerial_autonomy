#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"
#include <aerial_autonomy/common/conversions.h>
#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>

bool QrotorBacksteppingControllerConnector::extractSensorData(
    std::pair<double, QrotorBacksteppingState> &sensor_data) {

  std::chrono::duration<double> time_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - t_0_);
  double current_time = time_duration.count();
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  QrotorBacksteppingState current_state;

  if (odom_sensor_) {
    if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Pose sensor invalid!";
      return false;
    }
    auto quad_pose_velocity_pair = odom_sensor_->getSensorData();
    // Pose
    current_state.pose = quad_pose_velocity_pair.first;
    // RPY
    current_rpy_ = conversions::transformTfToRPY(current_state.pose);
    // Linear velocity
    current_state.v = quad_pose_velocity_pair.second;
  } else {
    // Pose
    current_state.pose = conversions::getPose(data);
    // RPY
    current_rpy_ =
        Eigen::Vector3d(data.rpydata.x, data.rpydata.y, data.rpydata.z);
    // Linear velocity
    current_state.v = tf::Vector3(data.linvel.x, data.linvel.y, data.linvel.z);
  }

  // Angular velocity
  current_state.w = tf::Vector3(data.omega.x, data.omega.y, data.omega.z);
  tf::vectorTFToEigen(current_state.w, current_omega_);

  // Thrust (Newton)
  current_state.thrust = thrust_;

  // Thrust_dot (Newton/sec)
  current_state.thrust_dot = thrust_dot_;

  sensor_data = std::make_pair(current_time, current_state);

  // Estimate thrust_gain
  thrust_gain_estimator_.addSensorData(current_rpy_(0), current_rpy_(1),
                                       data.linacc.z);

  return true;
}

void QrotorBacksteppingControllerConnector::sendControllerCommands(
    QrotorBacksteppingControl control) {
  Eigen::Matrix3d J; // Inertia matrix
  J << config_.jxx(), config_.jxy(), config_.jxz(), config_.jyx(),
      config_.jyy(), config_.jyz(), config_.jzx(), config_.jzy(), config_.jzz();
  double Thrust_ddot = control.thrust_ddot;
  tf::Vector3 Torque = control.torque;
  std::chrono::time_point<std::chrono::high_resolution_clock> current_time =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                previous_time_);
  double dt;
  if (initial_count_ == 0) {
    dt = 0.02;
    initial_count_++;
  } else {
    dt = dt_duration.count();
  }
  previous_time_ = current_time;
  thrust_dot_ += Thrust_ddot * dt;
  thrust_ += thrust_dot_ * dt;

  // conversions
  Eigen::Vector3d torque;
  tf::vectorTFToEigen(Torque, torque);
  Eigen::Vector3d omega_dot_cmd =
      J.llt().solve((J * current_omega_).cross(current_omega_) + torque);
  omega_command_ += omega_dot_cmd * dt;
  Eigen::Vector3d rpydot_cmd =
      conversions::omegaToRpyDot(omega_command_, current_rpy_);
  rpyt_message_.x += rpydot_cmd[0] * dt; // Roll command
  rpyt_message_.y += rpydot_cmd[1] * dt; // Pitch command
  rpyt_message_.z = rpydot_cmd[2];       // Yaw rate command

  // Bound command input
  rpyt_message_.x = math::clamp(rpyt_message_.x, lb_(0), ub_(0));
  rpyt_message_.y = math::clamp(rpyt_message_.y, lb_(1), ub_(1));
  rpyt_message_.z = math::clamp(rpyt_message_.z, lb_(2), ub_(2));
  thrust_ = math::clamp(thrust_, m_ * g_ * lb_(3), m_ * g_ * ub_(3));

  double thrust_cmd =
      thrust_ / (m_ * thrust_gain_estimator_.getThrustGain()); // thrust_command
  rpyt_message_.w = thrust_cmd;
  thrust_gain_estimator_.addThrustCommand(rpyt_message_.w);
  drone_hardware_.cmdrpyawratethrust(rpyt_message_);

  DATA_LOG("qrotor_backstepping_controller_connector")
      << current_rpy_(0) << current_rpy_(1) << current_rpy_(2)
      << rpyt_message_.x << rpyt_message_.y << rpyt_message_.z << thrust_
      << DataStream::endl;
}

void QrotorBacksteppingControllerConnector::setGoal(
    std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal) {
  BaseClass::setGoal(goal);
  goal_ = goal;
  t_0_ = std::chrono::high_resolution_clock::now();
  previous_time_ = std::chrono::high_resolution_clock::now();
  // Initial state
  thrust_ = m_ * g_;
  thrust_dot_ = 0;
  rpyt_message_.x = 0; // Initial roll
  rpyt_message_.y = 0; // Initial pitch
  omega_command_ << 0, 0, 0;
  // Set lower, upper bounds
  lb_ << -0.785, -0.785, -1.5708, 0.8;
  ub_ << 0.785, 0.785, 1.5708, 1.2;
  std::cout << "mass: " << config_.mass() << '\n'
            << "kp_xy: " << config_.kp_xy() << '\n'
            << "kd_xy: " << config_.kd_xy() << '\n'
            << "kp_z: " << config_.kp_z() << '\n'
            << "kd_z: " << config_.kd_z() << '\n'
            << "k1: " << config_.k1() << '\n'
            << "k2: " << config_.k2() << '\n'
            << "jxx: " << config_.jxx() << '\n'
            << "jyy: " << config_.jyy() << '\n'
            << "jzz: " << config_.jzz() << '\n';
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}

void QrotorBacksteppingControllerConnector::useSensor(
    SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> sensor) {
  odom_sensor_ = sensor;
}
