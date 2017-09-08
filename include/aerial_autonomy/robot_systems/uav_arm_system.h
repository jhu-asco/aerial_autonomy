#pragma once
#include "aerial_autonomy/common/math.h"
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
  /**
  * @brief Constructor
  * @param tracker Used to track targets for visual servoing
  * @param drone_hardware UAV driver
  * @param arm_hardware Manipulator driver
  * @param config Configuration parameters
  */
  UAVArmSystem(BaseTracker &tracker, parsernode::Parser &drone_hardware,
               ArmParser &arm_hardware, UAVSystemConfig config)
      : UAVVisionSystem(tracker, drone_hardware, config),
        ArmSystem(arm_hardware),
        grip_timeout_(config_.uav_vision_system_config()
                          .uav_arm_system_config()
                          .grip_timeout()),
        arm_goal_transforms_(
            math::getTransformsFromVector(config_.uav_vision_system_config()
                                              .uav_arm_system_config()
                                              .arm_goal_transform())),
        arm_transform_(
            math::getTransformFromVector(config_.uav_vision_system_config()
                                             .uav_arm_system_config()
                                             .arm_transform())),
        relative_pose_controller_(config_.uav_vision_system_config()
                                      .uav_arm_system_config()
                                      .position_controller_config()),
        visual_servoing_arm_connector_(tracker, drone_hardware, arm_hardware,
                                       relative_pose_controller_,
                                       camera_transform_, arm_transform_) {
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

  /**
   * @brief getter for timeout used for gripping
   *
   * @return timeout used for gripping
   */
  std::chrono::milliseconds gripTimeout() { return grip_timeout_; }

  /**
   * @brief getter for transfrom from arm to object
   * @param i Index of arm transform to get
   *
   * @return  transform from arm to object
   */
  tf::Transform armGoalTransform(int i) { return arm_goal_transforms_.at(i); }

private:
  /**
  * @brief Timeout for gripping by arm
  */
  std::chrono::milliseconds grip_timeout_;
  /**
  * @brief Arm transforms in the frame of the UAV
  */
  // \todo Matt Move to state machine config
  std::vector<tf::Transform> arm_goal_transforms_;
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
