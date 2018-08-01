#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"

bool QrotorBacksteppingControllerConnector::extractSensorData(
    std::pair<double, QrotorBacksteppingState> &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);

  double current_time = std::chrono::high_resolution_clock::now() - t_0_;

  QrotorBacksteppingState qrotor_backstepping_state;

  /* Objective
  *Use RPY & pos data from quaddata -> SE(3)
  * RPY -> tf::Quaternion;
  * pos -> tf::Vector3;
  * ex)pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0))
  */
  tf::Transform Pose;

  qrotor_backstepping_state.pose = Pose;
  // geometry_msgs::Vector3 linvel;   // Linear velocity of quadcopter
  qrotor_backstepping_state.v(data.linvel.x, data.linvel.y, data.linvel.z);

  // geometry_msgs::Vector3 omega;   // Angular velocities in NWU format
  qrotor_backstepping_state.w(data.omega.x, data.omega.y, data.omega.z);

  /* Objective
  * Acquire thrust, thrust_dot
  */
  qrotor_backstepping_state.thrust;

  qrotor_backstepping_state.thrust_dot;

  sensor_data = std::make_pair(current_time, qrotor_backstepping_state);

  return true;
}

void QrotorBacksteppingControllerConnector::sendControllerCommands(
    QrotorBacksteppingControl control);
{
  double Torque = control.torque;
  tf::Vector3 Thrust_ddot = control.thrust_ddot;
  /* Objective
  *Use torque, thrust_ddot -> get rpyratethrust
  */
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg
      .x // Roll
      rpyt_msg
      .y // Pitch
      rpyt_msg
      .z // Yaw rate
      rpyt_msg
      .w // thrust
      drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}
