#pragma once
#include "ros/ros.h" 
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "rpyt_controller_vins_connector_config.pb.h"
#include <parsernode/parser.h>
#include "aerial_autonomy/common/atomic.h"

/**
*
* @class RPYTControllerVINSConnector
*
* @brief Takes sensor data from VINS sensor to give rpyt commands to a drone 
*
**/
class RPYTControllerVINSConnector 
: public ControllerHardwareConnector <std::tuple<PositionYaw, VelocityYaw>, 
PositionYaw, RollPitchYawThrust>
{
public:
  /**
  * @brief Constructor
  *
  * Store hardware type as UAV.
  *
  * @param controller RPYT controller that achieves desired postion, yaw
  * 
  */
  RPYTControllerVINSConnector(
    parsernode::Parser &drone_hardware,
    Controller<std::tuple<PositionYaw, VelocityYaw>,PositionYaw,RollPitchYawThrust> &controller)
  :ControllerHardwareConnector(controller, HardwareType::UAV), 
  drone_hardware_(drone_hardware), 
  config_(RPYTControllerVINSConnectorConfig()){}

protected:
  /**
   * @brief  extracts pose, velocity and yaw data
   *
   * @param sensor_data Current position.velocity and yaw of UAV
   *
   * @return true if position, velocity and yaw can be extracted
   */
  virtual bool extractSensorData(std::tuple<PositionYaw, VelocityYaw> &sensor_data);

  /**
   * @brief  Send rpyt commands to hardware
   *
   * @param controls rpyt command to send to UAV
   */
  virtual void sendHardwareCommands(RollPitchYawThrust controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief Connector configuration 
  */
  RPYTControllerVINSConnectorConfig config_;
  /**
  * @brief sensor object to get sensor data 
  */
  Sensor<std::tuple<PositionYaw, VelocityYaw>> position_velocity_sensor_;
};
