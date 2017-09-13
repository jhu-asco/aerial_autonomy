#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "uav_vision_system_config.pb.h"

#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 */
class VisualServoingControllerDroneConnector
    : public ControllerHardwareConnector<PositionYaw, Position,
                                         VelocityYawRate> {
public:
  /**
   * @brief Constructor
   */
  VisualServoingControllerDroneConnector(
      BaseTracker &tracker, parsernode::Parser &drone_hardware,
      ConstantHeadingDepthController &controller,
      tf::Transform camera_transform)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        drone_hardware_(drone_hardware), tracker_(tracker),
        camera_transform_(camera_transform), explicit_id_(0),
        use_explicit_id_(false) {}
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingControllerDroneConnector() {}

  /**
   * @brief Get the tracking vector of the tracker in the global
   * frame
   * @param tracking_vector Returned tracking vector
   * @return True if successful and false otherwise
   */
  bool getTrackingVectorGlobalFrame(Position &tracking_vector);

  /**
  * @brief Set the tracker to use the id specified instead of tracking
  * strategy
  */
  void setExplicitId(uint32_t id) {
    explicit_id_ = id;
    use_explicit_id_ = true;
  }

  /**
  * @brief Remove explicit id and start using tracking strategy
  */
  void resetExplicitId() { use_explicit_id_ = false; }

protected:
  /**
   * @brief Extracts pose data from ROI
   *
   * @param sensor_data Position and yaw of object tracked by ROI
   *
   * @return true if able to extract ROI position and yaw
   */
  virtual bool extractSensorData(PositionYaw &sensor_data);

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
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief Tracks whatever we are servoing to
  */
  BaseTracker &tracker_;
  /**
  * @brief camera transform with respect to body
  */
  tf::Transform camera_transform_;

  /**
  * @brief if use_explicit_id is true will use this id instead
  * of tracking strategy
  */
  uint32_t explicit_id_;
  /**
  * @brief flag to specify whether to use the explicit id or
  * tracking strategy
  */
  bool use_explicit_id_;
};
