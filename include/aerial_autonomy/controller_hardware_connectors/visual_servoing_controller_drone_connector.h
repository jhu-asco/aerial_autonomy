#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/velocity_based_position_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "visual_servoing_controller_connector_config.pb.h"

#include <ros/ros.h>

#include <sensor_msgs/RegionOfInterest.h>

#include <parsernode/parser.h>

/**
 * @brief A visual servoing controller that uses an image ROI as feedback
 */
class VisualServoingControllerDroneConnector
    : public ControllerHardwareConnector<PositionYaw, PositionYaw,
                                         VelocityYaw> {
public:
  /**
   * @brief Constructor
   */
  VisualServoingControllerDroneConnector(
      ros::NodeHandle nh, parsernode::Parser &drone_hardware,
      VelocityBasedPositionController &controller,
      VisualServoingControllerConnectorConfig config)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        config_(config), drone_hardware_(drone_hardware), nh_(nh),
        roi_subscriber_(nh_.subscribe(
            "roi", 1, &VisualServoingControllerDroneConnector::roiCallback,
            this)) {}
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingControllerDroneConnector() {}

protected:
  /**
   * @brief Extracts pose data from ROI
   *
   * @return Position and yaw of object tracked by ROI
   */
  virtual PositionYaw extractSensorData();

  /**
   * @brief  Send velocity commands to hardware
   *
   * @param controls velocity command to send to UAV
   */
  virtual void sendHardwareCommands(VelocityYaw controls);

  /**
   * @brief ROI subscriber callback
   */
  void roiCallback(const sensor_msgs::RegionOfInterest &roi_msg);

private:
  /**
  * @brief Configuration
  */
  VisualServoingControllerConnectorConfig config_;
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief ROS node handle for receiving ROI
  */
  ros::NodeHandle nh_;
  /**
  * @brief ROS subscriber for receiving region of interest
  */
  ros::Subscriber roi_subscriber_;
};
