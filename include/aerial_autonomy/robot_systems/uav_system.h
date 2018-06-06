#pragma once

#include "uav_system_config.pb.h"
// Html Utilities
#include <aerial_autonomy/common/html_utils.h>
// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>
// Controllers
#include <aerial_autonomy/controllers/basic_controllers.h>
#include <aerial_autonomy/controllers/joystick_velocity_controller.h>
#include <aerial_autonomy/controllers/rpyt_based_position_controller.h>
// Estimators
#include <aerial_autonomy/estimators/thrust_gain_estimator.h>
// Specific ControllerConnectors
#include <aerial_autonomy/controller_connectors/basic_controller_connectors.h>
#include <aerial_autonomy/controller_connectors/joystick_velocity_controller_drone_connector.h>
// Sensors
#include <aerial_autonomy/controller_connectors/basic_controller_connectors.h>
#include <aerial_autonomy/controller_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controller_connectors/rpyt_based_position_controller_drone_connector.h>
#include <aerial_autonomy/controller_connectors/rpyt_based_position_controller_drone_connector.h>
#include <aerial_autonomy/sensors/guidance.h>
#include <aerial_autonomy/sensors/pose_sensor.h>
#include <aerial_autonomy/sensors/velocity_sensor.h>
// Load UAV parser
#include <pluginlib/class_loader.h>
// Base class for UAV parsers
#include <parsernode/parser.h>
// shared ptr
#include <memory>

#include <iomanip>
#include <sstream>

/**
 * @brief Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin position, velocity, and rpy controllers for controlling UAV
*/
class UAVSystem : public virtual BaseRobotSystem {
protected:
  /**
   * @brief typedef for base class of UAV parser
   */
  using UAVParserPtr = std::shared_ptr<parsernode::Parser>;
  /**
   * @brief UAV configuration parameters
   */
  UAVSystemConfig config_;
  /**
  * @brief Hardware
  */
  UAVParserPtr drone_hardware_;
  /**
  * @brief RPYT based position controller
  */
  RPYTBasedPositionController rpyt_based_position_controller_;
  /**
  * @brief Thrust gain estimator
  */
  ThrustGainEstimator thrust_gain_estimator_;

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
  * @brief Velocity controller which takes joystick controls as inputs
  */
  JoystickVelocityController joystick_velocity_controller_;

protected:
  /**
  * @brief Velocity Sensor
  */
  std::shared_ptr<Sensor<Velocity>> velocity_sensor_;
  /**
   * @brief pose_sensor_
   */
  std::shared_ptr<Sensor<tf::StampedTransform>> pose_sensor_;

private:
  /**
   * @brief connector for position controller
   */
  PositionControllerDroneConnector position_controller_drone_connector_;
  /**
   * @brief connector for position controller using rpyt
   * commands
   */
  RPYTBasedPositionControllerDroneConnector
      rpyt_based_position_controller_drone_connector_;
  /**
  * @brief connector for velocity controller
  */
  BuiltInVelocityControllerDroneConnector velocity_controller_drone_connector_;
  /**
  * @brief connector for rpyt controller
  */
  ManualRPYTControllerDroneConnector rpyt_controller_drone_connector_;

  /**
  * @brief Connector from JoystickVelocityController to drone hardware
  */
  JoystickVelocityControllerDroneConnector
      joystick_velocity_controller_drone_connector_;

  /**
  * @brief Home Location
  */
  PositionYaw home_location_;

  /**
  * @brief Flag to specify if home location is specified or not
  */
  bool home_location_specified_;
  /**
   * @brief helper function to choose between the argument parser
   * and the one provided in config. If user provided a parser,
   * that is used overwriting the one in config file
   *
   * @param parser The user provided parser or default null ptr
   * @param config UAV config containing parser type.
   *
   * @return the chosen parser
   */
  static UAVParserPtr chooseParser(UAVParserPtr parser,
                                   UAVSystemConfig &config) {
    UAVParserPtr uav_parser;
    if (parser) {
      uav_parser = parser;
    } else {
      pluginlib::ClassLoader<parsernode::Parser> parser_loader_(
          "parsernode", "parsernode::Parser");
      uav_parser = UAVParserPtr(
          parser_loader_.createUnmanagedInstance(config.uav_parser_type()));
    }
    return uav_parser;
  }
  /**
   * @brief function to choose type of sensor to use from config.
   * If user provides a sensor it overwrites this.
   *
   * @param sensor User provided parser
   * @param drone_hardware For sensors that require drone hardware
   * @param config UAV confif containing sensor type
   *
   */
  static std::shared_ptr<Sensor<Velocity>>
  chooseSensor(std::shared_ptr<Sensor<Velocity>> sensor,
               UAVParserPtr drone_hardware, UAVSystemConfig &config) {
    std::shared_ptr<Sensor<Velocity>> velocity_sensor;
    if (sensor)
      velocity_sensor = sensor;
    else {
      if (config.sensor_type() == "ROS Sensor") {
        velocity_sensor.reset(
            new VelocitySensor(config.velocity_sensor_config()));
      } else if (config.sensor_type() == "Guidance") {
        velocity_sensor.reset(new Guidance(*drone_hardware));
      } else {
        throw std::runtime_error("Invalid sensor type");
      }
    }

    return velocity_sensor;
  }
  /**
  * @brief create a pose sensor if using Motion Capture flag is set
  *
  * @param config The UAV system config
  *
  * @return new pose sensor if using motion capture otherwise nulllptr
  */
  static std::shared_ptr<Sensor<tf::StampedTransform>>
  createPoseSensor(UAVSystemConfig &config) {
    auto pose_sensor_config = config.pose_sensor_config();
    std::shared_ptr<Sensor<tf::StampedTransform>> pose_sensor;
    if (config.use_mocap_sensor()) {
      pose_sensor.reset(
          new PoseSensor(pose_sensor_config.topic(),
                         ros::Duration(pose_sensor_config.timeout())));
    }
    return pose_sensor;
  }

public:
  /**
   * @brief Constructor with default configuration
   */
  UAVSystem() : UAVSystem(UAVSystemConfig()) {}

  /**
   * @brief Constructor with hardware but no config
   *
   * @param drone_hardware explicitly provided drone hardware
   */
  UAVSystem(UAVParserPtr drone_hardware)
      : UAVSystem(UAVSystemConfig(), drone_hardware) {}
  /**
  * @brief Constructor
  *
  * UAVSystem with explicitly provided hardware. It instantiates the connectors,
  * controllers
  *
  * @param config The system configuration specifying the parameters such as
  * takeoff height, etc.
  *
  * @param drone_hardware input hardware to send commands back. If this variable
  * is set, it will overwrite the one given using "uav_parser_type" in config.
  */
  UAVSystem(UAVSystemConfig config, UAVParserPtr drone_hardware = nullptr,
            std::shared_ptr<Sensor<Velocity>> velocity_sensor = nullptr)
      : BaseRobotSystem(), config_(config),
        drone_hardware_(UAVSystem::chooseParser(drone_hardware, config)),
        rpyt_based_position_controller_(
            config.rpyt_based_position_controller_config(),
            std::chrono::milliseconds(config.uav_controller_timer_duration())),
        thrust_gain_estimator_(config.thrust_gain_estimator_config()),
        builtin_position_controller_(config.position_controller_config()),
        builtin_velocity_controller_(config.velocity_controller_config()),
        joystick_velocity_controller_(
            config.joystick_velocity_controller_config(),
            std::chrono::milliseconds(config.uav_controller_timer_duration())),
        velocity_sensor_(
            UAVSystem::chooseSensor(velocity_sensor, drone_hardware_, config)),
        pose_sensor_(UAVSystem::createPoseSensor(config)),
        position_controller_drone_connector_(*drone_hardware_,
                                             builtin_position_controller_),
        rpyt_based_position_controller_drone_connector_(
            *drone_hardware_, rpyt_based_position_controller_,
            thrust_gain_estimator_),
        velocity_controller_drone_connector_(*drone_hardware_,
                                             builtin_velocity_controller_),
        rpyt_controller_drone_connector_(*drone_hardware_,
                                         manual_rpyt_controller_),
        joystick_velocity_controller_drone_connector_(
            *drone_hardware_, joystick_velocity_controller_,
            thrust_gain_estimator_),
        home_location_specified_(false) {
    drone_hardware_->initialize();
    // Add control hardware connector containers
    controller_connector_container_.setObject(
        position_controller_drone_connector_);
    controller_connector_container_.setObject(
        rpyt_based_position_controller_drone_connector_);
    controller_connector_container_.setObject(
        velocity_controller_drone_connector_);
    controller_connector_container_.setObject(rpyt_controller_drone_connector_);
    controller_connector_container_.setObject(
        joystick_velocity_controller_drone_connector_);
  }
  /**
  * @brief Get sensor data from UAV
  *
  * @return Accumulated sensor data from UAV
  */
  parsernode::common::quaddata getUAVData() const {
    parsernode::common::quaddata data;
    drone_hardware_->getquaddata(data);
    return data;
  }

  /**
  * @brief Public API call to takeoff
  */
  void takeOff() { drone_hardware_->takeoff(); }

  /**
  * @brief Public API call to enable Quadcopter SDK.
  * This call is only necessary if Quad goes into manual mode
  * due to rc switching while state machine is running
  */
  void enableAutonomousMode() { drone_hardware_->flowControl(true); }

  /**
  * @brief Public API call to land
  */
  void land() { drone_hardware_->land(); }

  /**
  * @brief Provide the current state of UAV system
  *
  * @return string representation of the UAV system state
  */
  std::string getSystemStatus() const {
    parsernode::common::quaddata data = getUAVData();
    HtmlTableWriter table_writer;
    table_writer.beginRow();
    table_writer.addHeader("UAV Status", Colors::blue, 4);
    table_writer.beginRow();
    std::string battery_percent_color =
        (data.batterypercent > config_.minimum_battery_percent() ? Colors::green
                                                                 : Colors::red);
    table_writer.addCell(data.batterypercent, "Battery Percent",
                         battery_percent_color, 2);
    table_writer.beginRow();
    table_writer.addCell(data.localpos.x, "Local x");
    table_writer.addCell(data.localpos.y, "Local y");
    table_writer.addCell(data.localpos.z, "Local z");
    table_writer.addCell(data.altitude, "Altitude");
    table_writer.beginRow();
    table_writer.addCell(data.rpydata.x * (180 / M_PI), "Roll");
    table_writer.addCell(data.rpydata.y * (180 / M_PI), "Pitch");
    table_writer.addCell(data.rpydata.z * (180 / M_PI), "Yaw");
    table_writer.beginRow();
    table_writer.addCell(data.servo_in[0], "RC");
    table_writer.addCell(data.servo_in[1]);
    table_writer.addCell(data.servo_in[2]);
    table_writer.addCell(data.servo_in[3]);
    table_writer.beginRow();
    table_writer.addCell(data.linacc.x, "Acc x");
    table_writer.addCell(data.linacc.y, "Acc y");
    table_writer.addCell(data.linacc.z, "Acc z");
    table_writer.beginRow();
    table_writer.addCell(data.linvel.x, "Vel x");
    table_writer.addCell(data.linvel.y, "Vel y");
    table_writer.addCell(data.linvel.z, "Vel z");
    table_writer.beginRow();
    table_writer.addCell(data.velocity_goal.x, "Goal vel x");
    table_writer.addCell(data.velocity_goal.y, "Goal vel y");
    table_writer.addCell(data.velocity_goal.z, "Goal vel z");
    table_writer.addCell(data.velocity_goal_yaw, "Goal yaw");
    table_writer.beginRow();
    table_writer.addCell(data.position_goal.x, "Goal pos x");
    table_writer.addCell(data.position_goal.y, "Goal pos y");
    table_writer.addCell(data.position_goal.z, "Goal pos z");
    table_writer.addCell(data.velocity_goal_yaw, "Goal yaw");
    table_writer.beginRow();
    table_writer.addCell(data.mass, "Mass");
    table_writer.addCell(data.timestamp, "Timestamp", Colors::white, 2);
    table_writer.beginRow();
    std::string quad_state_color = (data.armed ? Colors::green : Colors::white);
    table_writer.addCell(data.quadstate, "Quadstate", quad_state_color, 2);
    return table_writer.getTableString();
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

  /**
   * @brief get the current pose frome either pose sensor/quad data
   * @return pose as stamped transform
   */
  tf::StampedTransform getPose() {
    tf::StampedTransform result;
    if (pose_sensor_) {
      result = pose_sensor_->getSensorData();
    } else {
      auto data = getUAVData();
      tf::Transform t(
          tf::createQuaternionFromRPY(data.rpydata.x, data.rpydata.y,
                                      data.rpydata.z),
          tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z));
      result =
          tf::StampedTransform(t, ros::Time(data.timestamp), "optitrak", "uav");
    }
    return result;
  }
};
