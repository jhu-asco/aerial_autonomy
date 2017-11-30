#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controller_hardware_connectors/base_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

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
          VelocityYawRate>,
      public BaseRelativePoseVisualServoingConnector {
public:
  /**
   * @brief Constructor
   * @param tracker Tracker to connect to controller
   * @param drone_hardware UAV hardware to send controller commands t
   * @param controller Controller to connect to UAV hardware
   * @param camera_transform Camera transform in UAV frame
   * @param tracking_offset_transform Additional transform to apply to tracked
   * object before it is roll/pitch compensated
   */
  RelativePoseVisualServoingControllerDroneConnector(
      BaseTracker &tracker, parsernode::Parser &drone_hardware,
      VelocityBasedRelativePoseController &controller,
      tf::Transform camera_transform,
      tf::Transform tracking_offset_transform = tf::Transform::getIdentity())
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        BaseRelativePoseVisualServoingConnector(tracker, drone_hardware,
                                                camera_transform,
                                                tracking_offset_transform) {}
  /**
   * @brief Destructor
   */
  virtual ~RelativePoseVisualServoingControllerDroneConnector() {}

protected:
  /**
   * @brief Extracts pose data from tracker
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
};
