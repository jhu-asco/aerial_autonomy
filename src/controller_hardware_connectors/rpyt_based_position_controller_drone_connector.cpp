#include "aerial_autonomy/controller_hardware_connectors/rpyt_based_position_controller_drone_connector.h"

bool RPYTBasedPositionControllerDroneConnector::extractSensorData(
    std::tuple<VelocityYawRate, PositionYaw> &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  PositionYaw position_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                           data.rpydata.z);
  VelocityYawRate velocity_yawrate(data.linvel.x, data.linvel.y, data.linvel.z,
                                   data.omega.z);
  sensor_data = std::make_tuple(velocity_yawrate, position_yaw);
  return true;
}

void RPYTBasedPositionControllerDroneConnector::sendHardwareCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}
