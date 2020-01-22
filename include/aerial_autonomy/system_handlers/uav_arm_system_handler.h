#pragma once

#include <ros/ros.h>

#include <aerial_autonomy/VelocityBasedPositionControllerDynamicConfig.h>
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/common/uav_ros_handle.h>
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
      : uav_system_(config.uav_system_config()), uav_ros_handle_(uav_system_),
        common_handler_(config.base_config(), uav_system_,
                        state_machine_config),
        uav_controller_timer_(std::bind(&UAVArmSystem::runActiveController,
                                        std::ref(uav_system_),
                                        ControllerGroup::UAV),
                              config.base_config().timer_multiplier() *
                                  std::chrono::milliseconds(
                                      config.uav_system_config()
                                          .uav_controller_timer_duration())),
        high_level_controller_timer_(
            std::bind(&UAVSystem::runActiveController, std::ref(uav_system_),
                      ControllerGroup::HighLevel),
            config.base_config().timer_multiplier() *
                std::chrono::milliseconds(
                    config.uav_system_config()
                        .uav_vision_system_config()
                        .high_level_controller_timer_duration())),
        arm_controller_timer_(std::bind(&UAVArmSystem::runActiveController,
                                        std::ref(uav_system_),
                                        ControllerGroup::Arm),
                              config.base_config().timer_multiplier() *
                                  std::chrono::milliseconds(
                                      config.uav_arm_system_handler_config()
                                          .arm_controller_timer_duration())),
        quad_mpc_visualization_timer_(
            std::bind(&UAVSystem::visualizeQuadMPC,
                      std::ref(this->uav_system_)),
            config.base_config().timer_multiplier() *
                std::chrono::milliseconds(
                    config.mpc_visualization_timer_duration())),
        state_publisher_timer_(
            std::bind(&UAVRosHandle::publish, std::ref(this->uav_ros_handle_)),
            config.base_config().timer_multiplier() *
                std::chrono::milliseconds(
                    config.state_publisher_timer_duration())) {

    // Get the party started
    common_handler_.startTimers();
    uav_controller_timer_.start();
    high_level_controller_timer_.start();
    arm_controller_timer_.start();
    if (config.uav_system_config().visualize_mpc_trajectories()) {
      quad_mpc_visualization_timer_.start();
    }
    state_publisher_timer_.start();
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
  UAVRosHandle uav_ros_handle_;
  CommonSystemHandler<LogicStateMachineT, EventManagerT, UAVArmSystem>
      common_handler_;              ///< Common logic to create state machine
                                    ///< and associated connections.
  AsyncTimer uav_controller_timer_; ///< Timer for running uav controller
  AsyncTimer high_level_controller_timer_; ///< Timer for running high level
  AsyncTimer arm_controller_timer_;        ///< Timer for running arm controller
  AsyncTimer quad_mpc_visualization_timer_; ///< Timer for visualizing MPC
  AsyncTimer state_publisher_timer_;        ///< Timer for publishing state
  MocapLogger mocap_logger_;                ///< Logger for mocap poses
};
