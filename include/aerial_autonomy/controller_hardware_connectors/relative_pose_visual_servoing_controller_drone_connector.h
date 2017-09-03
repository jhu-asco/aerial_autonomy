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
 * and brings the quadrotor
 * to a goal pose expressed in the tracked object's coordinate frame
 */
class RelativePoseVisualServoingControllerDroneConnector
    : public ControllerHardwareConnector<
          std::tuple<tf::Transform, tf::Transform>, PositionYaw,
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
   * @brief Get the tracking pose of the tracker in the rotation-compensated
   * frame of the quadrotor
   * @param tracking_vector Returned tracking pose
   * @return True if successful and false otherwise
   */
  bool getTrackingTransformRotationCompensatedQuadFrame(
      tf::Transform &tracking_transform);

protected:
  /**
   * @brief Extracts pose data from ROI
   *
   * @param sensor_data Current transform of quadrotor in the
   * rotation-compensated frame of the quadrotor
   * and tracking transform in the rotation-compensated frame of the quadrotor
   *
   * @return true if able to compute transforms
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
  tf::Transform getBodyFrameRotation();

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
