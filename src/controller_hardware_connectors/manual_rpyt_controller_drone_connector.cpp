#include "aerial_autonomy/controller_hardware_connectors/manual_rpyt_controller_drone_connector.h"

bool ManualRPYTControllerDroneConnector::extractSensorData(
    JoystickYaw &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  sensor_data = JoystickYaw(quad_data.servo_in[0], quad_data.servo_in[1],
                             quad_data.servo_in[2], quad_data.servo_in[3],
                             quad_data.rpydata.z);
  return true;
}

void ManualRPYTControllerDroneConnector::sendHardwareCommands(
    RollPitchYawThrust controls) {
  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  drone_hardware_.cmdrpythrust(rpyt_command, true);
}
