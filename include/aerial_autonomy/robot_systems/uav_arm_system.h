#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/mpc_trajectory_visualizer.h"
#include "aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h"
#include "aerial_autonomy/controller_connectors/visual_servoing_controller_arm_connector.h"
#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/controllers/relative_pose_controller.h"
#include "aerial_autonomy/robot_systems/arm_system.h"
#include "aerial_autonomy/robot_systems/uav_vision_system.h"
#include <chrono>

/**
* @brief UAV vision system with an arm.
*/
class UAVArmSystem : public UAVVisionSystem, public ArmSystem {
public:
  /**
  * @brief Constructor. If tracker, drone_hardware, arm_hardware are provided,
  * they will
  * overwrite the one provided in the configuration file
  * @param config Configuration parameters
  * @param tracker Used to track targets for visual servoing
  * @param drone_hardware UAV driver
  * @param arm_hardware Manipulator driver
  */
  UAVArmSystem(UAVSystemConfig config, BaseTrackerPtr tracker = nullptr,
               UAVParserPtr drone_hardware = nullptr,
               ArmParserPtr arm_hardware = nullptr)
      : UAVVisionSystem(config, tracker, drone_hardware),
        ArmSystem(config_.uav_vision_system_config()
                      .uav_arm_system_config()
                      .arm_system_config(),
                  arm_hardware),
        arm_transform_(
            conversions::protoTransformToTf(config_.uav_vision_system_config()
                                                .uav_arm_system_config()
                                                .arm_transform())),
        relative_pose_controller_(config_.uav_vision_system_config()
                                      .uav_arm_system_config()
                                      .position_controller_config()),
        visual_servoing_arm_connector_(
            *tracker_, *drone_hardware_, *arm_hardware_,
            relative_pose_controller_, camera_transform_, arm_transform_),
        mpc_controller_(
            config_.uav_vision_system_config()
                .uav_arm_system_config()
                .mpc_controller_config(),
            std::chrono::milliseconds(config_.uav_controller_timer_duration())),
        mpc_connector_(*drone_hardware_, *arm_hardware_, mpc_controller_,
                       thrust_gain_estimator_,
                       config.thrust_gain_estimator_config().buffer_size(),
                       pose_sensor_) {
    if (config_.uav_vision_system_config()
            .uav_arm_system_config()
            .visualize_mpc_trajectories()) {
      mpc_visualizer_.reset(new MPCTrajectoryVisualizer(
          mpc_connector_, config_.uav_vision_system_config()
                              .uav_arm_system_config()
                              .visualizer_config()));
    }
    controller_connector_container_.setObject(visual_servoing_arm_connector_);
    controller_connector_container_.setObject(mpc_connector_);
  }

  /**
  * @brief Provide the current state of UAV system
  *
  * @return string representation of the UAV system state
  */
  std::string getSystemStatus() const {
    std::stringstream status;
    status << UAVVisionSystem::getSystemStatus() << std::endl
           << ArmSystem::getSystemStatus();
    return status.str();
  }

  /**
  * @brief Helper function for tests to enforce fixed time difference
  * when differentiating the pose in MPC
  *
  * @param time_diff the fixed time difference to use
  */
  void usePerfectMPCTime(double time_diff = 0.02) {
    mpc_connector_.usePerfectTimeDiff(time_diff);
  }

  /**
  * @brief MPC trajectory visualization function
  */
  void visualizeMPC() {
    if (mpc_visualizer_) {
      mpc_visualizer_->publishTrajectory();
    }
  }

private:
  /**
  * @brief Arm transform in the frame of the UAV
  */
  tf::Transform arm_transform_;
  /**
  * @brief Moves to a position relative to a tracked position
  */
  RelativePoseController relative_pose_controller_;
  /**
  * @brief Connects relative position controller to tracker and arm
  */
  VisualServoingControllerArmConnector visual_servoing_arm_connector_;
  /**
   * @brief mpc_controller_
   */
  DDPAirmMPCController mpc_controller_;
  /**
   * @brief mpc_connector_
   */
  MPCControllerAirmConnector mpc_connector_;
  /**
   * @brief mpc trajectory visualizer
   */
  std::unique_ptr<MPCTrajectoryVisualizer> mpc_visualizer_;
};
