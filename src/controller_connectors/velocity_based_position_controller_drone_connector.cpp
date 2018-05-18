#include "aerial_autonomy/controller_connectors/velocity_based_position_controller_drone_connector.h"

bool VelocityBasedPositionControllerDroneConnector::extractSensorData(
    PositionYaw &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  sensor_data = PositionYaw(data.localpos.x, data.localpos.y, data.localpos.z,
                            data.rpydata.z);
  return true;
}

void VelocityBasedPositionControllerDroneConnector::sendControllerCommands(
    VelocityYawRate controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvel_yaw_rate_guided(velocity_cmd, controls.yaw_rate);
}
