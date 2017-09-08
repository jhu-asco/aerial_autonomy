#pragma once

#include <aerial_autonomy/GainsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/system_handlers/common_system_handler.h>

#include "uav_system_handler_config.pb.h"
#include <aerial_autonomy/sensors/base_sensor.h>
#include <aerial_autonomy/sensors/guidance.h>
#include <aerial_autonomy/sensors/velocity_sensor.h>

/**
 * @brief Owns all of the autonomous system components and is responsible for
 * thread management. Also owns a common system handler object to
 * handle creation of state machine and connecting it to GUI.
 *
 * @tparam LogicStateMachineT Logic state machine to use
 * @tparam EventManagerT Event manager to use
 */
template <class LogicStateMachineT, class EventManagerT>
class UAVSystemHandler {
public:
  /**
   * @brief Constructor
   * @param nh NodeHandle to use for event and command subscription
   * @param config Proto configuration parameters
   */
  UAVSystemHandler(UAVSystemHandlerConfig &config)
      : nh_uav_("~uav"), parser_loader_("parsernode", "parsernode::Parser"),
        uav_hardware_(
            parser_loader_.createUnmanagedInstance(config.uav_parser_type())),
        velocity_sensor_(
            config.sensor_type() == "ROS Sensor"
                ? dynamic_cast<Sensor<VelocityYaw> *>(new VelocitySensor(
                      *uav_hardware_, nh_uav_, config.velocity_sensor_config()))
                : (config.sensor_type() == "Guidance"
                       ? dynamic_cast<Sensor<VelocityYaw> *>(
                             new Guidance(*uav_hardware_))
                       : dynamic_cast<Sensor<VelocityYaw> *>(
                             new Sensor<VelocityYaw>()))),
        uav_system_(*uav_hardware_, config.uav_system_config(),
                    velocity_sensor_,
                    config.uav_controller_timer_duration() / 1000.0),
        common_handler_(config.base_config(), uav_system_),
        uav_controller_timer_(
            std::bind(&UAVSystem::runActiveController, std::ref(uav_system_),
                      HardwareType::UAV),
            std::chrono::milliseconds(config.uav_controller_timer_duration())),
        use_dynamic_reconfigure_(config.use_dynamic_reconfigure()) {
    // Set default dynamic reconfigure gains from configurationg
    callbacktype_ = boost::bind(&UAVSystemHandler::dynamicReconfigureCallback,
                                this, _1, _2);
    server_.setCallback(callbacktype_);
    aerial_autonomy::GainsConfig default_config;
    RPYTBasedVelocityControllerConfig start_config =
        config.uav_system_config()
            .joystick_velocity_controller_config()
            .rpyt_based_velocity_controller_config();
    default_config.kp = start_config.kp();
    default_config.ki = start_config.ki();
    default_config.kt = start_config.kt();
    server_.updateConfig(default_config);
    // Initialize UAV plugin
    uav_hardware_->initialize(nh_uav_);

    // Get the party started
    common_handler_.startTimers();
    uav_controller_timer_.start();
  }

  /**
  * @brief Delete copy constructor
  */
  UAVSystemHandler(const UAVSystemHandler &) = delete;

  /**
   * @brief Get UAV state
   * @return The UAV state
   */
  parsernode::common::quaddata getUAVData() {
    parsernode::common::quaddata quad_data;
    uav_hardware_->getquaddata(quad_data);
    return quad_data;
  }

  /**
  * @brief Forward common handler connected function for testing
  * is GUI is connected to this node or not
  *
  * @return true if connected
  */
  bool isConnected() { return common_handler_.isConnected(); }

private:
  ros::NodeHandle nh_uav_; ///< Nodehandle for UAV
  pluginlib::ClassLoader<parsernode::Parser>
      parser_loader_; ///< Used to load hardware plugin
  std::unique_ptr<parsernode::Parser> uav_hardware_; ///< Hardware instance
  std::shared_ptr<Sensor<VelocityYaw>>
      velocity_sensor_;  ///< External Velocity Sensor
  UAVSystem uav_system_; ///< Contains controllers
  CommonSystemHandler<LogicStateMachineT, EventManagerT, UAVSystem>
      common_handler_;              ///< Common logic to create state machine
                                    ///< and associated connections.
  AsyncTimer uav_controller_timer_; ///< Timer for running uav controller
  bool use_dynamic_reconfigure_;    ///< flag to use dynamic reconfigure
  dynamic_reconfigure::Server<aerial_autonomy::GainsConfig> server_;
  ///< dynamic reconfigure server
  dynamic_reconfigure::Server<aerial_autonomy::GainsConfig>::CallbackType
      callbacktype_; ///< dynamic reconfigure callbacktype

  /**
  * @brief Callback for dynamic reconfigure
  *
  * \todo soham create separate class for dynamic reconfigure
  */
  void dynamicReconfigureCallback(aerial_autonomy::GainsConfig &gains_config,
                                  uint32_t level) {
    if (use_dynamic_reconfigure_) {
      RPYTBasedVelocityControllerConfig config;
      config.set_kp(gains_config.kp);
      config.set_ki(gains_config.ki);
      config.set_kt(gains_config.kt);

      uav_system_.updateRPYTVelocityControllerConfig(config);
    } else {
      LOG(WARNING) << " 'Use dynamic reconfigure' flag set to false";
    }
  }
};