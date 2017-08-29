#pragma once
#include "aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h"
#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include "uav_system_config.pb.h"
/**
* @ brief UAV system with external sensor
*/
class UAVSensorSystem : public UAVSystem {
public:
  UAVSensorSystem(
      Sensor<VelocityYaw> &velocity_sensor, parsernode::Parser &drone_hardware,
      RPYTBasedVelocityControllerConfig &rpyt_velocity_controller_config)
      : UAVSensorSystem(velocity_sensor, drone_hardware, UAVSystemConfig(),
                        rpyt_velocity_controller_config,
                        JoystickVelocityControllerConfig()) {}
  /**
  *
  * @brief Constructor
  *
  * @param velocity_sensor sensor for manual velocity controller
  * @param drone_hardware UAV hardware
  * @param config Config file for UAVSystem
  */
  UAVSensorSystem(
      Sensor<VelocityYaw> &velocity_sensor, parsernode::Parser &drone_hardware,
      UAVSystemConfig config,
      RPYTBasedVelocityControllerConfig &rpyt_velocity_controller_config,
      JoystickVelocityControllerConfig joystick_velocity_controller_config)
      : UAVSystem(drone_hardware, config), velocity_sensor_(velocity_sensor),
        rpyt_velocity_controller_config_(rpyt_velocity_controller_config),
        joystick_velocity_controller_(rpyt_velocity_controller_config_,
                                      joystick_velocity_controller_config),
        joystick_velocity_controller_drone_connector_(
            drone_hardware, joystick_velocity_controller_, velocity_sensor_) {
    // Add hardware controllers to container
    controller_hardware_connector_container_.setObject(
        joystick_velocity_controller_drone_connector_);
  }

  SensorStatus getSensorStatus() { return velocity_sensor_.getSensorStatus(); }

private:
  /**
  * @brief sensor that gives velocity data
  */
  Sensor<VelocityYaw> &velocity_sensor_;
  /**
  * @brief Controller config for rpyt velocity controller
  */
  RPYTBasedVelocityControllerConfig &rpyt_velocity_controller_config_;
  /**
  * @brief Manual Velocity Controller
  */
  JoystickVelocityController joystick_velocity_controller_;
  /**
  * @brief Hardware connector for manual velocity controller
  */
  JoystickVelocityControllerDroneConnector
      joystick_velocity_controller_drone_connector_;
};
