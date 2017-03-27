#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"

PositionYaw VisualServoingControllerDroneConnector::extractSensorData() {
  return PositionYaw();
}

void VisualServoingControllerDroneConnector::sendHardwareCommands(
    VelocityYaw controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvelguided(velocity_cmd, controls.yaw);
}
