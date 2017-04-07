#include "aerial_autonomy/controller_hardware_connectors/builtin_velocity_controller_drone_connector.h"

bool BuiltInVelocityControllerDroneConnector::extractSensorData(
    VelocityYaw &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  sensor_data =
      VelocityYaw(data.linvel.x, data.linvel.y, data.linvel.z, data.rpydata.z);
  return true;
}

void BuiltInVelocityControllerDroneConnector::sendHardwareCommands(
    VelocityYaw controls) {
  geometry_msgs::Vector3 velocity_command;
  velocity_command.x = controls.x;
  velocity_command.y = controls.y;
  velocity_command.z = controls.z;
  drone_hardware_.cmdvelguided(velocity_command, controls.yaw);
}
