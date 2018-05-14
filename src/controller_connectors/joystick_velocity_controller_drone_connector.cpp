#include "aerial_autonomy/controller_connectors/joystick_velocity_controller_drone_connector.h"

JoystickVelocityControllerDroneConnector::
    JoystickVelocityControllerDroneConnector(
        parsernode::Parser &drone_hardware,
        JoystickVelocityController &controller,
        ThrustGainEstimator &thrust_gain_estimator)
    : ControllerConnector(controller, ControllerGroup::UAV),
      drone_hardware_(drone_hardware),
      thrust_gain_estimator_(thrust_gain_estimator),
      private_reference_controller_(controller) {}

void JoystickVelocityControllerDroneConnector::setGoal(EmptyGoal goal) {
  BaseClass::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}

bool JoystickVelocityControllerDroneConnector::extractSensorData(
    std::tuple<Joystick, VelocityYawRate, double> &sensor_data) {

  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);

  Joystick joy_data(quad_data.servo_in[0], quad_data.servo_in[1],
                    quad_data.servo_in[2], quad_data.servo_in[3]);

  VelocityYawRate vel_data(quad_data.linvel.x, quad_data.linvel.y,
                           quad_data.linvel.z, quad_data.omega.z);

  sensor_data = std::make_tuple(joy_data, vel_data, quad_data.rpydata.z);
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       quad_data.linacc.z);
  auto rpyt_controller_config = private_reference_controller_.getRPYTConfig();
  rpyt_controller_config.set_kt(thrust_gain_estimator_.getThrustGain());
  private_reference_controller_.updateRPYTConfig(rpyt_controller_config);
  return true;
}

void JoystickVelocityControllerDroneConnector::sendControllerCommands(
    RollPitchYawRateThrust controls) {

  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_command);
}
