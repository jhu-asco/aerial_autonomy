#pragma once
#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "uav_system_config.pb.h"

#include <ros/ros.h>

class UAVVisionSystem : public UAVSystem {
public:
  /**
  * @brief Constructor
  * @param nh ROS Nodehandle
  * @param drone_hardware UAV driver
  * @param config Configuration parameters
  */
  UAVVisionSystem(ros::NodeHandle &nh, parsernode::Parser &drone_hardware,
                  UAVSystemConfig config)
      : UAVSystem(drone_hardware, config),
        roi_to_position_converter_(nh),
        constant_heading_depth_controller_(config_.uav_vision_system_config()
                                               .visual_servoing_config()
                                               .controller_config()),
        visual_servoing_drone_connector_(
            roi_to_position_converter_, drone_hardware_, constant_heading_depth_controller_,
            config_.uav_vision_system_config().visual_servoing_config()) {
    controller_hardware_connector_container_.setObject(
        visual_servoing_drone_connector_);
  }

  /**
  * @brief Get the distance-scaled direction vector of a tracking ROI in the global frame
  * @param pos Returned position
  * @return True if successful and false otherwise
  */
  bool getTrackingVector(Position& pos) {
    return visual_servoing_drone_connector_.getTrackingVectorGlobalFrame(pos);
  }

private:
  RoiToPositionConverter roi_to_position_converter_;
  ConstantHeadingDepthController constant_heading_depth_controller_;
  VisualServoingControllerDroneConnector visual_servoing_drone_connector_;
};
