#include "aerial_autonomy/controller_hardware_connectors/rpyt_controller_vins_connector.h"
#include <tf/transform_datatypes.h>

bool RPYTControllerVINSConnector::extractSensorData(std::tuple<PositionYaw, VelocityYaw> &sensor_data)
{ 
  position_velocity_sensor_.getSensorData(sensor_data);
  return true;
}

void RPYTControllerVINSConnector::sendHardwareCommands(
  RollPitchYawThrust controls)
{
  geometry_msgs::Quaternion rpyt;
  rpyt.x = controls.r;
  rpyt.y = controls.p;
  rpyt.z = controls.y;
  rpyt.w = controls.t;
  drone_hardware_.cmdrpythrust(rpyt, true);
}
