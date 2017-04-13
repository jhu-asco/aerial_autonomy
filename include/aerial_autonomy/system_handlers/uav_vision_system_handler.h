#pragma once

#include <ros/ros.h>

#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>

#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
#include <aerial_autonomy/system_handlers/common_system_handler.h>
#include <aerial_autonomy/trackers/roi_to_position_converter.h>

#include "uav_system_handler_config.pb.h"

/**
 * @brief Owns all of the autonomous system components and is responsible for
 * thread management. Also owns a common system handler object to
 * handle creation of state machine and connecting it to GUI.
 *
 * @tparam LogicStateMachineT Logic state machine to use
 * @tparam EventManagerT Event manager to use
 */
template <class LogicStateMachineT, class EventManagerT>
class UAVVisionSystemHandler {
public:
  /**
   * @brief Constructor
   * @param nh NodeHandle to use for event and command subscription
   * @param config Proto configuration parameters
   */
  UAVVisionSystemHandler(ros::NodeHandle &nh, UAVSystemHandlerConfig &config)
      : parser_loader_("parsernode", "parsernode::Parser"),
        uav_hardware_(
            parser_loader_.createUnmanagedInstance(config.uav_parser_type())),
        roi_to_position_converter_(nh),
        uav_system_(roi_to_position_converter_, *uav_hardware_,
                    config.uav_system_config()),
        common_handler_(nh, config.base_config(), uav_system_),
        uav_controller_timer_(
            std::bind(&UAVVisionSystem::runActiveController,
                      std::ref(uav_system_), HardwareType::UAV),
            std::chrono::milliseconds(config.uav_controller_timer_duration())) {
    // Initialize UAV plugin
    uav_hardware_->initialize(nh);

    // Get the party started
    common_handler_.startTimers();
    uav_controller_timer_.start();
  }

  /**
  * @brief Delete copy constructor
  */
  UAVVisionSystemHandler(const UAVVisionSystemHandler &) = delete;

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
  pluginlib::ClassLoader<parsernode::Parser>
      parser_loader_; ///< Used to load hardware plugin
  std::unique_ptr<parsernode::Parser> uav_hardware_; ///< Hardware instance
  RoiToPositionConverter roi_to_position_converter_; ///< Tracking system
  UAVVisionSystem uav_system_;                       ///< Contains controllers
  CommonSystemHandler<LogicStateMachineT, EventManagerT, UAVVisionSystem>
      common_handler_;              ///< Common logic to create state machine
                                    ///< and associated connections.
  AsyncTimer uav_controller_timer_; ///< Timer for running uav controller
};
