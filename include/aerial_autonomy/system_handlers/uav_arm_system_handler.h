#pragma once

#include <ros/ros.h>

#include <aerial_autonomy/VelocityBasedPositionControllerDynamicConfig.h>
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/log/mocap_logger.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/system_handlers/common_system_handler.h>
#include <aerial_autonomy/trackers/alvar_tracker.h>
#include <aerial_autonomy/trackers/roi_to_position_converter.h>
#include <dynamic_reconfigure/server.h>

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
   * @param config Proto configuration parameters
   */
  UAVArmSystemHandler(UAVSystemHandlerConfig &config,
                      const BaseStateMachineConfig &state_machine_config)
      : uav_system_(config.uav_system_config()),
        common_handler_(config.base_config(), uav_system_,
                        state_machine_config),
        uav_controller_timer_(
            std::bind(&UAVArmSystem::runActiveController, std::ref(uav_system_),
                      ControllerGroup::UAV),
            std::chrono::milliseconds(
                config.uav_system_config().uav_controller_timer_duration())),
        high_level_controller_timer_(
            std::bind(&UAVSystem::runActiveController, std::ref(uav_system_),
                      ControllerGroup::HighLevel),
            std::chrono::milliseconds(
                config.uav_system_config()
                    .uav_vision_system_config()
                    .high_level_controller_timer_duration())),
        arm_controller_timer_(
            std::bind(&UAVArmSystem::runActiveController, std::ref(uav_system_),
                      ControllerGroup::Arm),
            std::chrono::milliseconds(config.uav_arm_system_handler_config()
                                          .arm_controller_timer_duration())) {

    // Get the party started
    common_handler_.startTimers();
    uav_controller_timer_.start();
    high_level_controller_timer_.start();
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
  parsernode::common::quaddata getUAVData() { return uav_system_.getUAVData(); }

  /**
  * @brief Forward common handler connected function for testing
  * is GUI is connected to this node or not
  *
  * @return true if connected
  */
  bool isConnected() { return common_handler_.isConnected(); }

protected:
  UAVArmSystem uav_system_; ///< Contains controllers

private:
  CommonSystemHandler<LogicStateMachineT, EventManagerT, UAVArmSystem>
      common_handler_;              ///< Common logic to create state machine
                                    ///< and associated connections.
  AsyncTimer uav_controller_timer_; ///< Timer for running uav controller
  AsyncTimer high_level_controller_timer_; ///< Timer for running high level
  AsyncTimer arm_controller_timer_;        ///< Timer for running arm controller
  MocapLogger mocap_logger_;               ///< Logger for mocap poses
};
