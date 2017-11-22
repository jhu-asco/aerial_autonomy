#include "aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h"

JoystickVelocityControllerDroneConnector::
    JoystickVelocityControllerDroneConnector(
        parsernode::Parser &drone_hardware,
        Controller<std::tuple<Joystick, VelocityYawRate, double>, EmptyGoal,
                   RollPitchYawRateThrust> &controller)
    : ControllerHardwareConnector(controller, HardwareType::UAV),
      drone_hardware_(drone_hardware) {}

bool JoystickVelocityControllerDroneConnector::extractSensorData(
    std::tuple<Joystick, VelocityYawRate, double> &sensor_data) {

  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);

  Joystick joy_data(quad_data.servo_in[0], quad_data.servo_in[1],
                    quad_data.servo_in[2], quad_data.servo_in[3]);

  VelocityYawRate vel_data(quad_data.linvel.x, quad_data.linvel.y,
                           quad_data.linvel.z, quad_data.omega.z);

  sensor_data = std::make_tuple(joy_data, vel_data, quad_data.rpydata.z);
  return true;
}

void JoystickVelocityControllerDroneConnector::sendHardwareCommands(
    RollPitchYawRateThrust controls) {

  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_command);
}
