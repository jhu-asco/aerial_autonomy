#include "aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h"
#include "joystick_velocity_controller_drone_connector_config.pb.h"

JoystickVelocityControllerDroneConnector::
    JoystickVelocityControllerDroneConnector(
        parsernode::Parser &drone_hardware,
        JoystickVelocityController &controller,
        ThrustGainEstimator &thrust_gain_estimator,
        std::shared_ptr<Sensor<Velocity>> velocity_sensor,
        JoystickVelocityControllerDroneConnectorConfig config)
    : ControllerHardwareConnector(controller, HardwareType::UAV),
      drone_hardware_(drone_hardware),
      thrust_gain_estimator_(thrust_gain_estimator),
      private_reference_controller_(controller),
      velocity_sensor_(velocity_sensor), config_(config) {}

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

  bool sensor_status =
      sensor_status_to_bool(velocity_sensor_->getSensorStatus());

  if (sensor_status) {
    Velocity velocity_sensor_data = velocity_sensor_->getSensorData();
    if (abs(velocity_sensor_data.x - quad_data.linvel.x) >
            config_.max_divergence() &&
        abs(velocity_sensor_data.y - quad_data.linvel.y) >
            config_.max_divergence() &&
        abs(velocity_sensor_data.z - quad_data.linvel.z) >
            config_.max_divergence()) {
      LOG(WARNING) << "Sensor deviates from quad data. Aborting Controller";
      return false;
    }
    VelocityYawRate vel_data(velocity_sensor_data, quad_data.omega.z);
    sensor_data = std::make_tuple(joy_data, vel_data, quad_data.rpydata.z);
    thrust_gain_estimator_.addSensorData(
        quad_data.rpydata.x, quad_data.rpydata.y, quad_data.linacc.z);
    auto rpyt_controller_config = private_reference_controller_.getRPYTConfig();
    rpyt_controller_config.set_kt(thrust_gain_estimator_.getThrustGain());
    private_reference_controller_.updateRPYTConfig(rpyt_controller_config);
  }
  return sensor_status;
}

void JoystickVelocityControllerDroneConnector::sendHardwareCommands(
    RollPitchYawRateThrust controls) {

  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_command);
}
