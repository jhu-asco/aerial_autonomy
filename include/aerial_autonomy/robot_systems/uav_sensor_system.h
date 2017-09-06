#pragma once
#include "aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h"
#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include "uav_sensor_system_config.pb.h"
/**
* @ brief UAV system with external velocity sensor
*/
class UAVSensorSystem : public UAVSystem {
public:
  /**
  *
  * @brief Constructor
  *
  * @param velocity_sensor Sensor for joystick velocity controller
  * @param drone_hardware UAV hardware
  * @param uav_system_config Config file for UAVSystem
  * @param rpyt_velocity_controller_config Config for rpyt velocity controller
  * @param joystick_velocity_controller_config Config for joystick controller
  * @param controller_timer_duration Timestep for controller
  *
  */
  UAVSensorSystem(Sensor<VelocityYaw> &velocity_sensor,
                  parsernode::Parser &drone_hardware,
                  UAVSensorSystemConfig uav_sensor_system_config,
                  Atomic<RPYTBasedVelocityControllerConfig>
                      &rpyt_velocity_controller_config,
                  double controller_timer_duration)
      : UAVSystem(drone_hardware, uav_sensor_system_config.uav_system_config()),
        velocity_sensor_(velocity_sensor),
        joystick_velocity_controller_(
            rpyt_velocity_controller_config,
            uav_sensor_system_config.joystick_velocity_controller_config(),
            controller_timer_duration),
        joystick_velocity_controller_drone_connector_(
            drone_hardware, joystick_velocity_controller_, velocity_sensor_) {
    // Add hardware controllers to container
    controller_hardware_connector_container_.setObject(
        joystick_velocity_controller_drone_connector_);
  }

  /**
  * @brief get status of the sensor
  */
  SensorStatus getSensorStatus() { return velocity_sensor_.getSensorStatus(); }

private:
  /**
* @brief sensor that gives velocity data
*/
  Sensor<VelocityYaw> &velocity_sensor_;
  /**
  * @brief Joystick Velocity Controller
  */
  JoystickVelocityController joystick_velocity_controller_;
  /**
  * @brief Hardware connector for joystick velocity controller
  */
  JoystickVelocityControllerDroneConnector
      joystick_velocity_controller_drone_connector_;
};
