#include "aerial_autonomy/controller_hardware_connectors/rpyt_controller_vins_connector.h"
#include <tf/transform_datatypes.h>

void RPYTControllerVINSConnector::sensorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
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
    std::tuple<PositionYaw, VelocityYaw> curr_sensor_data;
    curr_sensor_data = sensor_data_;
    VelocityYaw vel_data;
    PositionYaw curr_pos_data = std::get<0>(curr_sensor_data);
    vel_data.x = (global_pos[0] - curr_pos_data.x)/dt;
    vel_data.y = (global_pos[1] - curr_pos_data.y)/dt;
    vel_data.z = (global_pos[2] - curr_pos_data.z)/dt;

    PositionYaw pos_data(global_pos[0], global_pos[1], global_pos[2], 0);

  // Convert to global frame
    tf::Quaternion q_s = tf::Quaternion(msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);

    tf::Quaternion q = sensor_tf.inverse()*q_s;

    double r,p,y;
    tf::Matrix3x3 m(q);
    m.getRPY(r,p,y);

    vel_data.yaw = y;
    pos_data.yaw = y;

    std::tuple<PositionYaw, VelocityYaw> sensor_data = std::make_tuple(pos_data, vel_data);
    sensor_data_ = sensor_data;
  }
  else
    ROS_WARN("VINS data diverges from GPS by more than %f", config_.max_divergence());
  
}

bool RPYTControllerVINSConnector::extractSensorData(std::tuple<PositionYaw, VelocityYaw> &sensor_data)
{ 
  sensor_data = sensor_data_;
  return true;
}

void RPYTControllerVINSConnector::sendHardwareCommands(
  RollPitchYawThrust controls)
{
  geometry_msgs::Quaternion rpyt;
  rpyt.x = controls.r;
  rpyt.y = controls.p;
  rpyt.z = controls.y;
  rpyt.w = controls.t;
  drone_hardware_.cmdrpythrust(rpyt, true);
}
