#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"

bool QrotorBacksteppingControllerConnector::extractSensorData(
    std::pair<double, QrotorBacksteppingState> &sensor_data) {
  // parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data_);

  // double current_time = std::chrono::high_resolution_clock::now() - t_0_;
  std::chrono::duration<double> time_duration = duration_cast<duration<double>>(
      std::chrono::high_resolution_clock::now() - t_0_);
  double current_time = time_duration.count();
  QrotorBacksteppingState qrotor_backstepping_state;

  /* Objective
  *Use RPY & pos data from quaddata -> SE(3)
  * RPY -> tf::Quaternion;
  * pos -> tf::Vector3;
  * ex)pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0))
  */
  // geometry_msgs::Vector3 rpydata; // Roll pitch yaw data in NWU format
  tf::Quaternion q = tf::createQuaternionFromRPY(
      data_.rpydata.x, data_.rpydata.y, data_.rpydata.z);

  // geometry_msgs::Vector3 localpos; // Local pos based on home NWU format
  qrotor_backstepping_state.pose(
      q, tf::Vector3(data_.localpos.x, data_.localpos.y, data_.localpos.z));

  // geometry_msgs::Vector3 linvel;   // Linear velocity of quadcopter
  qrotor_backstepping_state.v(data_.linvel.x, data_.linvel.y, data_.linvel.z);

  // geometry_msgs::Vector3 omega;   // Angular velocities in NWU format
  qrotor_backstepping_state.w(data_.omega.x, data_.omega.y, data_.omega.z);

  /* Objective
  * Acquire thrust, thrust_dot
  */
  // geometry_msgs::Vector3 linacc;   // Linear acceleration of quadcopter
  // data.linacc.z;
  qrotor_backstepping_state.thrust = thrust_;

  qrotor_backstepping_state.thrust_dot = thrust_dot_;

  sensor_data = std::make_pair(current_time, qrotor_backstepping_state);

  thrust_gain_estimator_.addSensorData(data_.rpydata.x, data_.rpydata.y,
                                       data_.linacc.z);

  return true;
}

void QrotorBacksteppingControllerConnector::sendControllerCommands(
    QrotorBacksteppingControl control,
    std::pair<double, QrotorBacksteppingState> &sensor_data);
{
  double Thrust_ddot = control.thrust_ddot;
  tf::Vector3 Torque = control.torque;
  /* Objective
  *Use torque, thrust_ddot -> get rpyratethrust
  */
  thrust_dot_ = Thrust_ddot * dt_;
  thrust_ = thrust_dot_ * dt_;

  QrotorBacksteppingState current_state = std::get<1> sensor_data;
  // conversions
  Eigen::Vector3d torque;
  tf::vectorTFToEigen(Torque, torque);
  Eigen::Vector3d current_omega;
  tf::vectorTFToEigen(current_state.omega, current_omega);
  Eigen::Vector3d current_rpy;
  current_rpy << data_rpydata.x, data_rpydata.y, data_rpydata.z;

  Eigen::Vector3d omega_dot_cmd =
      J_.llt().solve(J_ * current_omega.cross(current_omega) + torque);
  Eigen::Vector3 omega_cmd = omega_dot_cmd * dt_;

  Eigen::Vector3d rpydot_cmd = omegaToRpyDot(omega_cmd, current_rpy);

  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = rpydot_cmd[0] * dt_; // Roll command
  rpyt_msg.y = rpydot_cmd[1] * dt_; // Pitch command
  rpyt_msg.z = rpydot_cmd[2];       // Yaw rate command
  rpyt_msg.w =
      thrust_ / (m_ * thrust_gain_estimator_.getThrustGain()); // thrust_command
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

void QrotorBacksteppingControllerConnector::setGoal(
    std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal) {
  BaseClass::setGoal(goal);
  t_0_ = std::chrono::high_resolution_clock::now();
  thrust_ = m_ * g_;
  thrust_dot_ = 0.0;
  // setThrust(config_.mass() * config_.acc_gravity());
  // setThrustDot(0.0);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}

Eigen::Vector3d QrotorBacksteppingControllerConnector::omegaToRpyDot(
    const Eigen::Vector3d &omega, const Eigen::Vector3d &rpy) {
  Eigen::Matrix3d Mrpy;
  double s_roll = sin(rpy[0]), c_roll = cos(rpy[0]), t_pitch = tan(rpy[1]);
  double sec_pitch = sqrt(1 + t_pitch * t_pitch);
  Mrpy << 1, s_roll * t_pitch, c_roll * t_pitch, 0, c_roll, -s_roll, 0,
      s_roll * sec_pitch, c_roll * sec_pitch;
  return Mrpy * omega;
}
