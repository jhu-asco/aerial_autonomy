#include "aerial_autonomy/controller_hardware_connectors/rpyt_controller_vins_connector.h"
#include <tf/transform_datatypes.h>

void RPYTControllerVINSConnector::sensorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Get gps data for checking if pose is diverging
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  double dt = 0.03;

  // Convert to global frame
  tf::Vector3 pos = tf::Vector3(msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);

  tf::Vector3 global_pos = sensor_tf*pos;

  if(abs(msg->pose.position.x - data.localpos.x) < 0.5 ||
    abs(msg->pose.position.y- data.localpos.y) < 0.5 ||
    abs(msg->pose.position.z - data.localpos.z) < 0.5)
  {
  // Differentiate position to get velocity
    VelocityYaw vel_data;
    PositionYaw curr_pos_data = std::get<0>(sensor_data_);
    vel_data.x = (global_pos[0] - curr_pos_data.x)/dt;
    vel_data.y = (global_pos[1] - curr_pos_data.y)/dt;
    vel_data.z = (global_pos[2] - curr_pos_data.z)/dt;

    PositionYaw pos_data;
    pos_data.x = global_pos[0];
    pos_data.y = global_pos[1];
    pos_data.z = global_pos[2];

  // Convert to global frame
    tf::Quaternion q_s = tf::Quaternion(msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);

    tf::Quaternion q = sensor_tf*q_s;

    double r,p,y;
    tf::Matrix3x3 m(q);
    m.getRPY(r,p,y);

    vel_data.yaw = y;
    pos_data.yaw = y;

    std::get<0>(sensor_data_) = pos_data;
    std::get<1>(sensor_data_) = vel_data;
  }
}

bool RPYTControllerVINSConnector::extractSensorData(std::tuple<PositionYaw, VelocityYaw> &sensor_data)
{ 
  sensor_data_ = sensor_data;
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