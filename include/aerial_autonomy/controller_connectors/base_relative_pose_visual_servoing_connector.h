#pragma once
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and brings the quadrotor
 * to a goal pose expressed in the tracked object's coordinate frame
 */
class BaseRelativePoseVisualServoingConnector {
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
  BaseRelativePoseVisualServoingConnector(
      BaseTracker &tracker, parsernode::Parser &drone_hardware,
      tf::Transform camera_transform, tf::Transform tracking_offset_transform)
      : drone_hardware_(drone_hardware), tracker_(tracker),
        camera_transform_(camera_transform),
        // \todo Matt This will become unwieldy when we are tracking multiple
        // objects, each with different offsets.  This assumes the offset is the
        // same for all tracked objects
        tracking_offset_transform_(tracking_offset_transform) {}
  /**
   * @brief Destructor
   */
  virtual ~BaseRelativePoseVisualServoingConnector() {}

  /**
   * @brief Get the rotation-compensated tracking pose of the tracker in the
   * rotation-compensated
   * frame of the quadrotor
   * @param obect_pose_cam Transform of the object in the camera's frame
   * @parm body_frame_rotation quad orientation in inertial frame
   * @return Returned tracking pose in rotation compensated frame
   */
  tf::Transform getTrackingTransformRotationCompensatedQuadFrame(
      tf::Transform object_pose_cam, tf::Transform body_frame_rotation);
  /**
  * @brief Get the angle between camera z and the marker center
  *
  * @param object_pose_cam the transform of marker in camera frame
  *
  * @return angle in radians
  */
  double getViewingAngle(tf::Transform object_pose_cam) const;

protected:
  /**
   * @brief logTrackerData
   *
   * Log tracker info to stream
   * @param stream_name name of logging stream
   * @param tracking_pose Tracker pose
   * @param object_pose_cam Object pose in camera frame
   * @param quad_data Quadrotor sensor data
   */
  void logTrackerData(std::string stream_name, tf::Transform tracking_pose,
                      tf::Transform object_pose_cam,
                      parsernode::common::quaddata &quad_data);

  /**
   * @brief logTrackerHeader
   *
   * Log tracker header to stream
   * @param stream_name name of logging stream
   */
  void logTrackerHeader(std::string stream_name);

protected:
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
  /**
  * @brief transform to apply to tracked object in its own frame before
  * roll/pitch compensation
  */
  tf::Transform tracking_offset_transform_;
};
