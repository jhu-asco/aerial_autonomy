#pragma once

#include "uav_system_config.pb.h"
// Html Utilities
#include <aerial_autonomy/common/html_utils.h>
// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>
// Controllers
#include <aerial_autonomy/controllers/basic_controllers.h>
// Specific ControllerConnectors
#include <aerial_autonomy/controller_hardware_connectors/basic_controller_hardware_connectors.h>
// Velocity Sensor
#include <aerial_autonomy/sensors/base_sensor.h>
// System Id
#include <gcop/qrotorsystemid.h>
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
  * @brief Hardware
  */
  parsernode::Parser &drone_hardware_;
  /**
   * @brief UAV configuration parameters
   */
  UAVSystemConfig config_;
  /**
  * @brief Controller timestep
  */
  double controller_timer_duration_;

private:
  /**
  * @brief velocity sensor
  */
  std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>
      velocity_pose_sensor_;
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
  * @brief joystick velocity controller
  */
  JoystickVelocityController joystick_velocity_controller_;
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
  * @brief Joystick Velocity controller
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
  * @ brief Variable to store measurements for system id
  */
  std::vector<gcop::QRotorSystemIDMeasurement> system_id_measurements;
  /**
  * @brief Sytem Id object
  */
  gcop::QRotorSystemID system_id;

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
  * @param velocity_sensor Velocity Sensor to be used by robot system
  * @param controller_timer_duration Timestep for controllers
  */
  UAVSystem(parsernode::Parser &drone_hardware, UAVSystemConfig config,
            std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>
                velocity_pose_sensor =
                    std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
                        new Sensor<std::tuple<VelocityYaw, Position>>()),
            double controller_timer_duration = 0.02)
      : BaseRobotSystem(), drone_hardware_(drone_hardware), config_(config),
        controller_timer_duration_(controller_timer_duration),
        velocity_pose_sensor_(velocity_pose_sensor),
        builtin_position_controller_(config.position_controller_config()),
        builtin_velocity_controller_(config.velocity_controller_config()),
        manual_rpyt_controller_(config.manual_rpyt_controller_config(),
                                controller_timer_duration),
        joystick_velocity_controller_(
            config.joystick_velocity_controller_config(),
            controller_timer_duration),
        position_controller_drone_connector_(drone_hardware,
                                             builtin_position_controller_),
        velocity_controller_drone_connector_(drone_hardware,
                                             builtin_velocity_controller_),
        rpyt_controller_drone_connector_(drone_hardware,
                                         manual_rpyt_controller_),
        joystick_velocity_controller_drone_connector_(
            drone_hardware, joystick_velocity_controller_,
            *velocity_pose_sensor),
        home_location_specified_(false) {
    // Add control hardware connector containers
    controller_hardware_connector_container_.setObject(
        position_controller_drone_connector_);
    controller_hardware_connector_container_.setObject(
        velocity_controller_drone_connector_);
    controller_hardware_connector_container_.setObject(
        rpyt_controller_drone_connector_);
    controller_hardware_connector_container_.setObject(
        joystick_velocity_controller_drone_connector_);

    DATA_HEADER("system_id_measurements") << "Timestamp"
                                          << "PositionX"
                                          << "PositionY"
                                          << "PositionZ"
                                          << "Roll "
                                          << "Pitch"
                                          << "Yaw"
                                          << "CommandedThrust"
                                          << "CommandedRoll"
                                          << "CommandedPitch"
                                          << "CommandedYawRate"
                                          << DataStream::endl;
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
  * @brief Get status of the sensor
  */
  SensorStatus getVelocityPoseSensorStatus() {
    return velocity_pose_sensor_->getSensorStatus();
  }

  /**
  * @brief Get data from sensor
  */
  std::tuple<VelocityYaw, Position> getVelocityPoseSensorData() {
    return velocity_pose_sensor_->getSensorData();
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
    table_writer.addCell(data.magdata.x, "Mag x");
    table_writer.addCell(data.magdata.y, "Mag y");
    table_writer.addCell(data.magdata.z, "Mag z");
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
    table_writer.addCell(data.servo_in[0], "Channel 1");
    table_writer.addCell(data.servo_in[1], "Channel 2");
    table_writer.addCell(data.servo_in[2], "Channel 3");
    table_writer.addCell(data.servo_in[3], "Channel 4");
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
  * @brief update the rpyt controller config
  *
  * \todo soham make fuction templated and
  * extend for other controllers
  */
  void updateRPYTVelocityControllerConfig(
      RPYTBasedVelocityControllerConfig &config) {
    joystick_velocity_controller_.updateRPYTConfig(config);
  }
  /**
  * @brief get the rpyt controller config
  */
  RPYTBasedVelocityControllerConfig getRPYTVelocityControllerConfig() {
    return joystick_velocity_controller_.getRPYTConfig();
  }
  /**
  * @brief get last commanded yaw
  * /todo soham remove after implementing estimators class
  */
  double getLastCommandedYaw() {
    return manual_rpyt_controller_.getLastCommandedYaw();
  }
  /**
  * @brief set last commanded yaw
  *
  * @param manual_controller Flag to check which controller to set for
  * true: set for velocity-yaw controller
  * false: set for manual rpyt controller
  *
  * \todo soham replace by reset function in controller
  */
  void setLastCommandedYaw(double last_commanded_yaw,
                           bool manual_controller = false) {
    if (manual_controller)
      manual_rpyt_controller_.setLastCommandedYaw(last_commanded_yaw);
    else
      joystick_velocity_controller_.setLastCommandedYaw(last_commanded_yaw);
  }
  /**
  * @brief add measurements for system id
  */
  void addMeasurement(gcop::QRotorSystemIDMeasurement measurement) {
    DATA_LOG("system_id_measurements")
        << measurement.t << measurement.position[0] << measurement.position[1]
        << measurement.position[2] << measurement.rpy[0] << measurement.rpy[1]
        << measurement.rpy[2] << measurement.control[0]
        << measurement.control[1] << measurement.control[2]
        << measurement.control[3] << DataStream::endl;

    system_id_measurements.push_back(measurement);
  }
  /**
  * @brief reset measurements
  */
  void clearMeasurements() { system_id_measurements.clear(); }
  /**
  * @brief run system id
  */
  void runSystemId() {
    if (system_id_measurements.size() <= 101) {
      // \todo soham make it a parameter
      LOG(WARNING) << "Too few measurements for system id. Exiting.";
      return;
    }

    std::vector<gcop::QRotorSystemIDMeasurement> estimator_measurements;
    int iterations = system_id_measurements.size() / 100;
    std::cout << "no of iterations = " << iterations << std::endl;

    for (int k = 0; k < iterations; k++) {
      gcop::QRotorIDState init_state;
      init_state.p = system_id_measurements[k * 100].position;
      const Eigen::Vector3d &rpy = system_id_measurements[k * 100].rpy;

      gcop::SO3 &so3 = gcop::SO3::Instance();
      so3.q2g(init_state.R, rpy);
      init_state.u << 0, 0, rpy(2);

      estimator_measurements.resize(100);
      // \todo soham make both these parameters
      for (int j = 0; j < 100; j++) {
        estimator_measurements[j].t = system_id_measurements[k * 100 + j + 1].t;
        estimator_measurements[j].position =
            system_id_measurements[k * 100 + j + 1].position;
        estimator_measurements[j].rpy =
            system_id_measurements[k * 100 + j + 1].rpy;
        // convert to rolldot, pitchdot
        estimator_measurements[j].control
            << system_id_measurements[k * 100 + j + 1].control[0],
            (system_id_measurements[k * 100 + j + 1].control[1] -
             system_id_measurements[k * 100 + j].control[1]) /
                controller_timer_duration_,
            (system_id_measurements[k * 100 + j + 1].control[2] -
             system_id_measurements[k * 100 + j].control[2]) /
                controller_timer_duration_,
            system_id_measurements[k * 100 + j + 1].control[3];
      }
      // Run estimator
      system_id.offsets_timeperiod = 0.5; // \todo soham make it a param
      system_id.EstimateParameters(estimator_measurements, init_state);
      estimator_measurements.clear();
      // Update gain only if gain is within bounds
      if ((system_id.qrotor_gains_lb[0] < system_id.qrotor_gains[0]) &
          (system_id.qrotor_gains_ub[0] > system_id.qrotor_gains[0])) {
        RPYTBasedVelocityControllerConfig config;
        VLOG(1) << "kt changed to " << system_id.qrotor_gains[0] << "\n";
        config.set_kt(system_id.qrotor_gains[0]);
        updateRPYTVelocityControllerConfig(config);
      } else {
        LOG(WARNING) << "Gain out of bounds";
      }
    }
  }

  /**
  * @brief returns the controller timestep
  */
  double getControllerTimerDuration() { return controller_timer_duration_; }
};
