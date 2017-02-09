#include "aerial_autonomy/controller_hardware_connectors/builtin_velocity_controller_drone_connector.h"

EmptySensor BuiltInVelocityControllerDroneConnector::extractSensorData() {
  return EmptySensor();
}

void BuiltInVelocityControllerDroneConnector::sendHardwareCommands(
    VelocityYaw controls) {
  geometry_msgs::Vector3 velocity_command;
  velocity_command.x = controls.x;
  velocity_command.y = controls.y;
  velocity_command.z = controls.z;
  drone_hardware_.cmdvelguided(velocity_command, controls.yaw);
}
