#include "aerial_autonomy/controller_hardware_connectors/manual_velocity_controller_drone_connector.h"

void ManualVelocityControllerDroneConnector::sensorCallback(
  const geometry_msgs::PoseStamped::ConstPtr& msg){

 // Get gps data for checking if pose is diverging
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);

  double dt = config_.dt();

  // Convert to global frame
  tf::Vector3 pos = tf::Vector3(msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);

  tf::Vector3 global_pos = sensor_tf.inverse()*pos;

  if(abs(global_pos[0] - data.localpos.x) < config_.max_divergence() ||
    abs(global_pos[1]- data.localpos.y) < config_.max_divergence() ||
    abs(global_pos[2] - data.localpos.z) < config_.max_divergence())
  {
    // Differentiate position to get velocity
    VelocityYaw vel_sensor_data;

    vel_sensor_data.x = (global_pos[0] - last_pos.x)/dt;
    vel_sensor_data.y = (global_pos[1] - last_pos.y)/dt;
    vel_sensor_data.z = (global_pos[2] - last_pos.z)/dt;

    last_pos.x = global_pos[0];
    last_pos.y = global_pos[1];
    last_pos.z = global_pos[2];

      // Convert to global frame
    tf::Quaternion q_s = tf::Quaternion(msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);

    tf::Quaternion q = sensor_tf.inverse()*q_s;

    double r,p,y;
    tf::Matrix3x3 m(q);
    m.getRPY(r,p,y);

    vel_sensor_data.yaw = y;

    vel_sensor_data_ = vel_sensor_data;
  }
}

bool ManualVelocityControllerDroneConnector::extractSensorData(
  std::tuple<JoysticksYaw, VelocityYaw> &sensor_data) {

  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);

  JoysticksYaw joy_data;
  joy_data = JoysticksYaw(quad_data.servo_in[0], quad_data.servo_in[1],
   quad_data.servo_in[2], quad_data.servo_in[3],
   quad_data.rpydata.z);

    VelocityYaw vel_sensor_data = vel_sensor_data_;
    sensor_data = std::make_tuple(joy_data, vel_sensor_data); 
  return true;
}

void ManualVelocityControllerDroneConnector::sendHardwareCommands(
  RollPitchYawThrust controls) {

  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  drone_hardware_.cmdrpythrust(rpyt_command, true);
}
