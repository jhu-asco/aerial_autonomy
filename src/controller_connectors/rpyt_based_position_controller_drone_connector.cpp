#include "aerial_autonomy/controller_connectors/rpyt_based_position_controller_drone_connector.h"

bool RPYTBasedPositionControllerDroneConnector::extractSensorData(
    std::tuple<VelocityYawRate, PositionYaw> &sensor_data) {
  if (sensor_) {
    if (sensor_->getSensorStatus() == SensorStatus::INVALID) {
      return false;
    }
    sensor_data = sensor_->getTransformedSensorData();
  }
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  if (!sensor_) {
    PositionYaw position_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                             data.rpydata.z);
    VelocityYawRate velocity_yawrate(data.linvel.x, data.linvel.y,
                                     data.linvel.z, data.omega.z);
    sensor_data = std::make_tuple(velocity_yawrate, position_yaw);
  }
  tf::Vector3 body_acc(data.linacc.x, data.linacc.y, data.linacc.z);
  thrust_gain_estimator_.addSensorData(data.rpydata.x, data.rpydata.y,
                                       body_acc);
  auto rpyt_controller_config = private_reference_controller_.getRPYTConfig();
  rpyt_controller_config.set_kt(thrust_gain_estimator_.getThrustGain());
  private_reference_controller_.updateRPYTConfig(rpyt_controller_config);
  return true;
}

void RPYTBasedPositionControllerDroneConnector::sendControllerCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  Eigen::Vector2d roll_pitch_bias = thrust_gain_estimator_.getRollPitchBias();
  rpyt_msg.x = controls.r - roll_pitch_bias[0];
  rpyt_msg.y = controls.p - roll_pitch_bias[1];
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

void RPYTBasedPositionControllerDroneConnector::setGoal(PositionYaw goal) {
  BaseClass::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}
