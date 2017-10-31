#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_arm_connector.h"
#include "aerial_autonomy/controllers/relative_pose_controller.h"
#include "aerial_autonomy/robot_systems/arm_system.h"
#include "aerial_autonomy/robot_systems/uav_vision_system.h"
#include <chrono>

/**
* @brief UAV vision system with an arm.
*/
class UAVArmSystem : public UAVVisionSystem, public ArmSystem {
public:
  UAVArmSystem(UAVSystemConfig config)
      : UAVArmSystem(nullptr, nullptr, nullptr, config) {}
  /**
  * @brief Constructor
  * @param tracker Used to track targets for visual servoing
  * @param drone_hardware UAV driver
  * @param arm_hardware Manipulator driver
  * @param config Configuration parameters
  */
  UAVArmSystem(BaseTrackerPtr tracker, ParserPtr drone_hardware,
               ArmParserPtr arm_hardware, UAVSystemConfig config)
      : UAVVisionSystem(tracker, drone_hardware, config),
        ArmSystem(arm_hardware, config_.uav_vision_system_config()
                                    .uav_arm_system_config()
                                    .arm_system_config()),
        arm_transform_(
            conversions::protoTransformToTf(config_.uav_vision_system_config()
                                                .uav_arm_system_config()
                                                .arm_transform())),
        relative_pose_controller_(config_.uav_vision_system_config()
                                      .uav_arm_system_config()
                                      .position_controller_config()),
        visual_servoing_arm_connector_(
            *tracker_, *drone_hardware_, *arm_hardware_,
            relative_pose_controller_, camera_transform_, arm_transform_) {
    controller_hardware_connector_container_.setObject(
        visual_servoing_arm_connector_);
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
};
