#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "uav_vision_system_config.pb.h"

#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 */
class RelativePoseVisualServoingControllerDroneConnector
    : public ControllerHardwareConnector<
          std::tuple<tf::Transform, tf::Transform>, tf::Transform,
          VelocityYawRate> {
public:
  /**
   * @brief Constructor
   */
  RelativePoseVisualServoingControllerDroneConnector(
      BaseTracker &tracker, parsernode::Parser &drone_hardware,
      VelocityBasedRelativePoseController &controller,
      tf::Transform camera_transform)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        drone_hardware_(drone_hardware), tracker_(tracker),
        camera_transform_(camera_transform) {}
  /**
   * @brief Destructor
   */
  virtual ~RelativePoseVisualServoingControllerDroneConnector() {}

  /**
   * @brief Get the tracking vector of the tracker in the global
   * frame
   * @param tracking_vector Returned tracking vector
   * @return True if successful and false otherwise
   */
  bool getTrackingTransformRotationCompensatedQuadFrame(
      tf::Transform &tracking_transform);

protected:
  /**
   * @brief Extracts pose data from ROI
   *
   * @param sensor_data Position and yaw of object tracked by ROI
   *
   * @return true if able to extract ROI position and yaw
   */
  virtual bool
  extractSensorData(std::tuple<tf::Transform, tf::Transform> &sensor_data);

  /**
   * @brief Send velocity commands to hardware
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
};
