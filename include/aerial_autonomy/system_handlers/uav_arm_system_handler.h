#pragma once

#include <ros/ros.h>

#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>

#include <arm_parsers/arm_simulator.h>
#include <arm_parsers/generic_arm.h>
#include <arm_parsers/simple_arm.h>

#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/system_handlers/common_system_handler.h>
#include <aerial_autonomy/trackers/alvar_tracker.h>
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
class UAVArmSystemHandler {
public:
  /**
   * @brief Constructor
   * @param nh NodeHandle to use for event and command subscription
   * @param config Proto configuration parameters
   */
  UAVArmSystemHandler(UAVSystemHandlerConfig &config,
                      const BaseStateMachineConfig &state_machine_config)
      : nh_uav_("~uav"), nh_arm_("~arm"), nh_tracker_("~tracker"),
        parser_loader_("parsernode", "parsernode::Parser"),
        uav_hardware_(
            parser_loader_.createUnmanagedInstance(config.uav_parser_type())),
        // \todo Make arm hardware a plugin
        arm_hardware_(
            config.uav_arm_system_handler_config().arm_parser_type() ==
                    "GenericArm"
                ? dynamic_cast<ArmParser *>(new GenericArm(nh_arm_))
                : (config.uav_arm_system_handler_config().arm_parser_type() ==
                           "SimpleArm"
                       ? dynamic_cast<ArmParser *>(new SimpleArm(nh_arm_))
                       : dynamic_cast<ArmParser *>(new ArmSimulator()))),
        tracker_(
            config.uav_system_config()
                        .uav_vision_system_config()
                        .tracker_type() == "ROI"
                ? dynamic_cast<BaseTracker *>(
                      new RoiToPositionConverter(nh_tracker_))
                : dynamic_cast<BaseTracker *>(new AlvarTracker(nh_tracker_))),
        uav_system_(*tracker_, *uav_hardware_, *arm_hardware_,
                    config.uav_system_config()),
        common_handler_(config.base_config(), uav_system_,
                        state_machine_config),
        uav_controller_timer_(
            std::bind(&UAVArmSystem::runActiveController, std::ref(uav_system_),
                      HardwareType::UAV),
            std::chrono::milliseconds(config.uav_controller_timer_duration())),
        arm_controller_timer_(
            std::bind(&UAVArmSystem::runActiveController, std::ref(uav_system_),
                      HardwareType::Arm),
            std::chrono::milliseconds(config.uav_arm_system_handler_config()
                                          .arm_controller_timer_duration())) {
    // Initialize UAV plugin
    uav_hardware_->initialize(nh_uav_);

    // Get the party started
    common_handler_.startTimers();
    uav_controller_timer_.start();
    arm_controller_timer_.start();
  }

  /**
  * @brief Delete copy constructor
  */
  UAVArmSystemHandler(const UAVArmSystemHandler &) = delete;

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
  ros::NodeHandle nh_uav_;     ///< Nodehandle for UAV
  ros::NodeHandle nh_arm_;     ///< Nodehandle for Arm
  ros::NodeHandle nh_tracker_; ///< Nodehandle for Tracker
  pluginlib::ClassLoader<parsernode::Parser>
      parser_loader_; ///< Used to load hardware plugin
  std::unique_ptr<parsernode::Parser> uav_hardware_; ///< Hardware instance
  std::unique_ptr<ArmParser> arm_hardware_;          ///< Arm Hardware instance
  std::unique_ptr<BaseTracker> tracker_;             ///< Tracking system
  UAVArmSystem uav_system_;                          ///< Contains controllers
  CommonSystemHandler<LogicStateMachineT, EventManagerT, UAVArmSystem>
      common_handler_;              ///< Common logic to create state machine
                                    ///< and associated connections.
  AsyncTimer uav_controller_timer_; ///< Timer for running uav controller
  AsyncTimer arm_controller_timer_; ///< Timer for running arm controller
};
