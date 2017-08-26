#pragma once
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "uav_system_config.pb.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "aerial_autonomy/controllers/manual_velocity_controller.h"
#include "aerial_autonomy/controller_hardware_connectors/manual_velocity_controller_drone_connector.h"
#include "rpyt_based_velocity_controller_config.pb.h"
/**
* @ brief UAV system with external sensor
*/
class UAVSensorSystem : public UAVSystem{
public:
  UAVSensorSystem(Sensor<VelocityYaw> &velocity_sensor,
    parsernode::Parser &drone_hardware, 
    RPYTBasedVelocityControllerConfig &rpyt_vel_ctlr_config) :
  UAVSensorSystem(velocity_sensor, 
    drone_hardware,
    UAVSystemConfig(), 
    rpyt_vel_ctlr_config
    ){}
/**
* 
* @brief Constructor
*
* @param velocity_sensor sensor for manual velocity controller 
* @param drone_hardware UAV hardware
* @param config Config file for UAVSystem
*/
  UAVSensorSystem(Sensor<VelocityYaw> &velocity_sensor,
    parsernode::Parser &drone_hardware, 
    UAVSystemConfig config,
    RPYTBasedVelocityControllerConfig &rpyt_vel_ctlr_config) 
  : UAVSystem(drone_hardware, config),
  velocity_sensor_(velocity_sensor),
  rpyt_vel_ctlr_config_(rpyt_vel_ctlr_config),
  manual_velocity_controller_(rpyt_vel_ctlr_config_),
  manual_velocity_controller_drone_connector_(
    drone_hardware,
    manual_velocity_controller_,
    velocity_sensor_){
        // Add hardware controllers to container
    controller_hardware_connector_container_.setObject(
      manual_velocity_controller_drone_connector_);
  }

  SensorStatus getSensorStatus(){
    return velocity_sensor_.getSensorStatus();
  } 

private:
  /**
  * @brief sensor that gives velocity data
  */
  Sensor<VelocityYaw> &velocity_sensor_;
  /**
  * @brief Controller config for 
  */
  RPYTBasedVelocityControllerConfig &rpyt_vel_ctlr_config_;
  /**
  * @brief Manual Velocity Controller
  */  
  ManualVelocityController manual_velocity_controller_;
  /**
  * @brief Hardware connector for manual velocity controller
  */
  ManualVelocityControllerDroneConnector manual_velocity_controller_drone_connector_;
};