#pragma once

#include "uav_system_config.pb.h"
// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>
// Controllers
#include <aerial_autonomy/controllers/basic_controllers.h>
// Specific ControllerConnectors
#include <aerial_autonomy/controller_hardware_connectors/basic_controller_hardware_connectors.h>

/**
 * @brief Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin position, velocity, and rpy controllers for controlling UAV
*/
class UAVSystem : public BaseRobotSystem {
protected:
  /**
  * @brief Hardware
  */
  parsernode::Parser &drone_hardware_;
  /**
   * @brief UAV configuration parameters
   */
  UAVSystemConfig config_;

private:
  // Controllers
  /**
  * @brief Position Controller
  */
  BuiltInPositionController builtin_position_controller_;
  /**
  * @brief  velocity controller
  */
  BuiltInVelocityController builtin_velocity_controller_;
  /**
  * @brief rpyt controller using joystick controller connectors
  */
  ManualRPYTController manual_rpyt_controller_;
  /**
  * @brief velocity controller using joystick controller connector
  */
  ManualVelocityController manual_velocity_controller_;
  /**
   * @brief connector for position controller
   */
  PositionControllerDroneConnector position_controller_drone_connector_;
  /**
  * @brief connector for velocity controller
  */
  BuiltInVelocityControllerDroneConnector velocity_controller_drone_connector_;
  /**
  * @brief connector for rpyt controller
  */
  ManualRPYTControllerDroneConnector rpyt_controller_drone_connector_;
  /**
  * brief connector for manual velocity controller
  */
  ManualVelocityControllerDroneConnector manual_velocity_controller_drone_connector_;
  /**
  * @brief Home Location
  */
  PositionYaw home_location_;

  /**
  * @brief Flag to specify if home location is specified or not
  */
  bool home_location_specified_;

public:
  /**
   * @brief Constructor with default configuration
   */
  UAVSystem(parsernode::Parser &drone_hardware)
      : UAVSystem(drone_hardware, UAVSystemConfig()) {}
  /**
  * @brief Constructor
  *
  * UAVSystem requires a drone hardware. It instantiates the connectors,
  * controllers
  *
  * @param drone_hardware input hardware to send commands back
  * @param config The system configuration specifying the parameters such as
  * takeoff height, etc.
  */
  UAVSystem(parsernode::Parser &drone_hardware, UAVSystemConfig config)
      : BaseRobotSystem(), drone_hardware_(drone_hardware), config_(config),
        builtin_position_controller_(config.position_controller_config()),
        builtin_velocity_controller_(config.velocity_controller_config()),
        position_controller_drone_connector_(drone_hardware,
                                             builtin_position_controller_),
        velocity_controller_drone_connector_(drone_hardware,
                                             builtin_velocity_controller_),
        rpyt_controller_drone_connector_(drone_hardware,
                                         manual_rpyt_controller_),
        manual_velocity_controller_drone_connector_(drone_hardware,
                                          manual_velocity_controller_),
        home_location_specified_(false) {
    // Add control hardware connector containers
    controller_hardware_connector_container_.setObject(
        position_controller_drone_connector_);
    controller_hardware_connector_container_.setObject(
        velocity_controller_drone_connector_);
    controller_hardware_connector_container_.setObject(
        rpyt_controller_drone_connector_);
    controller_hardware_connector_container_.setObject(
        manual_velocity_controller_drone_connector_);
  }
  /**
  * @brief Get sensor data from UAV
  *
  * @return Accumulated sensor data from UAV
  */
  parsernode::common::quaddata getUAVData() const {
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);
    return data;
  }

  /**
  * @brief Public API call to takeoff
  */
  void takeOff() { drone_hardware_.takeoff(); }

  /**
  * @brief Public API call to enable Quadcopter SDK.
  * This call is only necessary if Quad goes into manual mode
  * due to rc switching while state machine is running
  */
  void enableAutonomousMode() { drone_hardware_.flowControl(true); }

  /**
  * @brief Public API call to land
  */
  void land() { drone_hardware_.land(); }

  /**
  * @brief Provide the current state of UAV system
  *
  * @return string representation of the UAV system state
  */
  std::string getSystemStatus() const {
    parsernode::common::quaddata data = getUAVData();
    /// \todo Matt fix this to avoid stack overflows
    char buffer[1500];
    sprintf(
        buffer,
        "Battery Percent: %2.2f\t\nLocal x: %2.2f\tLocal y: %2.2f\tLocal z: "
        "%2.2f\nAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\n"
        "Mag x: %2.2f\tMag y %2.2f\tMag z %2.2f\nAcc x: %2.2f\tAcc y "
        "%2.2f\tAcc z %2.2f\nVel x: %2.2f\tVel y %2.2f\tVel z %2.2f\n"
        "Goal vx: %2.2f\tGoal vy: %2.2f\tGoal vz: %2.2f\t Goal vyaw: %2.2f\n"
        "Goal x: %2.2f\tGoal y: %2.2f\tGoal z: %2.2f\t Goal yaw: %2.2f\n"
        "Mass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s",
        data.batterypercent, data.localpos.x, data.localpos.y, data.localpos.z,
        data.altitude, data.rpydata.x * (180 / M_PI),
        data.rpydata.y * (180 / M_PI),
        data.rpydata.z * (180 / M_PI), // IMU rpy angles
        data.magdata.x, data.magdata.y, data.magdata.z, data.linacc.x,
        data.linacc.y, data.linacc.z, data.linvel.x, data.linvel.y,
        data.linvel.z, data.velocity_goal.x, data.velocity_goal.y,
        data.velocity_goal.z, data.velocity_goal_yaw, data.position_goal.x,
        data.position_goal.y, data.position_goal.z, data.position_goal_yaw,
        data.mass, data.timestamp, data.quadstate.c_str());
    return buffer;
  }

  /**
   * @brief Get system configuration
   * @return Configuration
   */
  UAVSystemConfig getConfiguration() { return config_; }

  /**
  * @brief save current location as home location
  */
  void setHomeLocation() {
    parsernode::common::quaddata data = getUAVData();
    home_location_.x = data.localpos.x;
    home_location_.y = data.localpos.y;
    home_location_.z = data.localpos.z;
    home_location_.yaw = data.rpydata.z;
    home_location_specified_ = true;
  }

  /**
  * @brief Check if home location is specified
  *
  * @return True if home location is specified
  */
  bool isHomeLocationSpecified() { return home_location_specified_; }

  /**
  * @brief Stored home location
  *
  * @return Home location (PositionYaw)
  */
  PositionYaw getHomeLocation() { return home_location_; }
};
