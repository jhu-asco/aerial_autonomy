#pragma once
#include "aerial_autonomy/common/roi_to_position_converter.h"
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "visual_servoing_controller_connector_config.pb.h"

#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses an image ROI as feedback
 */
class VisualServoingControllerDroneConnector
    : public ControllerHardwareConnector<PositionYaw, Position,
                                         VelocityYawRate> {
public:
  /**
   * @brief Constructor
   */
  VisualServoingControllerDroneConnector(
      RoiToPositionConverter &roi_to_position_converter, parsernode::Parser &drone_hardware,
      ConstantHeadingDepthController &controller,
      VisualServoingControllerConnectorConfig config)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        config_(config), drone_hardware_(drone_hardware),
        roi_to_position_converter_(roi_to_position_converter) {}
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingControllerDroneConnector() {}

  /**
   * @brief Get the tracking vector of the RoiToPositionConverter in the global
   * frame
   * @param tracking_vector Returned tracking vector
   * @return True if successful and false otherwise
   */
  bool getTrackingVectorGlobalFrame(Position& tracking_vector);

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
  virtual void sendHardwareCommands(VelocityYawRate controls);

private:
  /**
   * @brief Get the rotation of the uav body frame
   * @return The rotation transform
   */
  tf::Matrix3x3 getBodyFrameRotation();

  /**
  * @brief Configuration
  */
  VisualServoingControllerConnectorConfig config_;
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief Converts received ROS ROI to a position
  */
  RoiToPositionConverter &roi_to_position_converter_;
  /*
  * @brief camera transform with respect to body
  * \todo Matt Add to configuration (using rpy maybe?)
  */
  tf::Transform camera_transform_;
};
