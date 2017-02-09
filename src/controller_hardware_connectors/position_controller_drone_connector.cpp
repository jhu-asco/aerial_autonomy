#include "aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h"

EmptySensor PositionControllerDroneConnector::extractSensorData() {
  return EmptySensor();
}

void PositionControllerDroneConnector::sendHardwareCommands(PositionYaw controls) {
  geometry_msgs::Vector3 position_command;
  position_command.x = controls.x;
  position_command.y = controls.y;
  position_command.z = controls.z;
  drone_hardware_.cmdwaypoint(position_command, controls.yaw);
}
