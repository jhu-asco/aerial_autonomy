#include "aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h"

JoystickVelocityControllerDroneConnector::
    JoystickVelocityControllerDroneConnector(
        parsernode::Parser &drone_hardware,
        Controller<std::tuple<Joystick, VelocityYaw>, EmptyGoal,
                   RollPitchYawThrust> &controller,
        Sensor<VelocityYaw> &velocity_sensor)
    : ControllerHardwareConnector(controller, HardwareType::UAV),
      drone_hardware_(drone_hardware), velocity_sensor_(velocity_sensor) {
  drone_hardware_.setmode("rpyt_angle");
}

bool JoystickVelocityControllerDroneConnector::extractSensorData(
    std::tuple<Joystick, VelocityYaw> &sensor_data) {

  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);

  Joystick joy_data(quad_data.servo_in[0], quad_data.servo_in[1],
                    quad_data.servo_in[2], quad_data.servo_in[3]);

  if (velocity_sensor_.getSensorStatus() == SensorStatus::INVALID) {
    LOG(WARNING) << "Velocity sensor data invalid !";
  } else {
    VelocityYaw vel_sensor_data = velocity_sensor_.getSensorData();
    sensor_data = std::make_tuple(joy_data, vel_sensor_data);
  }

  return bool(velocity_sensor_.getSensorStatus());
}

void JoystickVelocityControllerDroneConnector::sendHardwareCommands(
    RollPitchYawThrust controls) {

  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  drone_hardware_.cmdrpythrust(rpyt_command, true);
}
