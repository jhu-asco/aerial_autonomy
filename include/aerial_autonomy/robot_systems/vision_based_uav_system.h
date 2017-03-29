#pragma once
#include <aerial_autnomy/robot_systems/uav_system.h>

class VisionBasedUAVSystem : public UAVSystem {
public:
  VisionBasedUAVSystem(ros::NodeHandle& nh, parsernode::Parser& drone_hardware, UAVSystemConfig config) 
      : UAVSystem(drone_hardware, config,
        nh_(nh),
        constant_heading_depth_controller_(config_.vision_based_uav_config().visual_servoing_config().controller_config())
        visual_servoing_drone_connector_(nh, drone_hardware_, constant_heading_depth_controller_, config_.vision_based_uav_config().visual_servoing_config()) {
    controller_hardware_connector_container_.setObject(
        visual_servoing_drone_connector_);
  }
protected:
  ros::NodeHandle& nh_;
private:
  ConstantHeadingDepthController constant_heading_depth_controller_;
  VisualServoingControllerDroneConnector visual_servoing_drone_connector_;
};
