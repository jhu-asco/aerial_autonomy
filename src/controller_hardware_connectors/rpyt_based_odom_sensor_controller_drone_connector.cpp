#include "aerial_autonomy/controller_hardware_connectors/rpyt_based_odom_sensor_controller_drone_connector.h"

bool RPYTBasedOdomSensorControllerDroneConnector::extractSensorData(
    std::tuple<VelocityYawRate, PositionYaw> &sensor_data) {
  if (sensor_.getSensorStatus() == SensorStatus::INVALID) {
    return false;
  }
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  sensor_data = sensor_.getSensorData();
  VelocityYawRate vel = std::get<0>(sensor_data);
  PositionYaw pos = std::get<1>(sensor_data);
  DATA_LOG("rpyt_based_odom_sensor_controller_drone_connector")
      << vel.x << vel.y << vel.z << vel.yaw_rate
      << pos.x << pos.y << pos.z << pos.yaw << DataStream::endl;
  thrust_gain_estimator_.addSensorData(data.rpydata.x, data.rpydata.y,
                                       data.linacc.z);
  auto rpyt_controller_config = private_reference_controller_.getRPYTConfig();
  rpyt_controller_config.set_kt(thrust_gain_estimator_.getThrustGain());
  private_reference_controller_.updateRPYTConfig(rpyt_controller_config);
  return true;
}

void RPYTBasedOdomSensorControllerDroneConnector::sendHardwareCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  DATA_LOG("rpyt_based_odom_sensor_controller_drone_connector_controls")
      << controls.r << controls.p << controls.y << controls.t << DataStream::endl;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

void RPYTBasedOdomSensorControllerDroneConnector::setGoal(PositionYaw goal) {
  BaseClass::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}
