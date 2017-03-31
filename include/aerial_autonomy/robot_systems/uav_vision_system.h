#pragma once
#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "uav_system_config.pb.h"

#include <ros/ros.h>

class UAVVisionSystem : public UAVSystem {
public:
  UAVVisionSystem(ros::NodeHandle &nh, parsernode::Parser &drone_hardware,
                  UAVSystemConfig config)
      : UAVSystem(drone_hardware, config),
        constant_heading_depth_controller_(config_.uav_vision_system_config()
                                               .visual_servoing_config()
                                               .controller_config()),
        visual_servoing_drone_connector_(
            nh, drone_hardware_, constant_heading_depth_controller_,
            config_.uav_vision_system_config().visual_servoing_config()) {
    controller_hardware_connector_container_.setObject(
        visual_servoing_drone_connector_);
  }

private:
  ConstantHeadingDepthController constant_heading_depth_controller_;
  VisualServoingControllerDroneConnector visual_servoing_drone_connector_;
};
