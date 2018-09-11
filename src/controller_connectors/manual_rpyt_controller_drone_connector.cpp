#include "aerial_autonomy/controller_connectors/manual_rpyt_controller_drone_connector.h"
#include "aerial_autonomy/log/log.h"

ManualRPYTControllerDroneConnector::ManualRPYTControllerDroneConnector(
    parsernode::Parser &drone_hardware,
    Controller<Joystick, EmptyGoal, RollPitchYawRateThrust> &controller)
    : ControllerConnector(controller, ControllerGroup::UAV),
      drone_hardware_(drone_hardware) {
  DATA_HEADER("manual_rpyt_controller") << "Roll"
                                        << "Pitch"
                                        << "Yaw"
                                        << "Omegax"
                                        << "Omegay"
                                        << "Omegaz"
                                        << "BatteryPercent"
                                        << "Accx"
                                        << "Accy"
                                        << "Accz" << DataStream::endl;
}

bool ManualRPYTControllerDroneConnector::extractSensorData(
    Joystick &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  DATA_LOG("manual_rpyt_connector")
      << quad_data.rpydata.x << quad_data.rpydata.y << quad_data.rpydata.z
      << quad_data.omega.x << quad_data.omega.y << quad_data.omega.z
      << quad_data.batterypercent << quad_data.linacc.x << quad_data.linacc.y
      << quad_data.linacc.z << DataStream::endl;
  sensor_data = Joystick(quad_data.servo_in[0], quad_data.servo_in[1],
                         quad_data.servo_in[2], quad_data.servo_in[3]);
  return true;
}

void ManualRPYTControllerDroneConnector::sendControllerCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_command;
  rpyt_command.x = controls.r;
  rpyt_command.y = controls.p;
  rpyt_command.z = controls.y;
  rpyt_command.w = controls.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_command);
}
