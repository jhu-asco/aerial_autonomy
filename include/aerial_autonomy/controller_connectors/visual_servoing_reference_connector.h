#pragma once
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controller_connectors/base_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/reference_trajectory.h"

#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and brings the quadrotor
 * to a goal pose expressed in the tracked object's coordinate frame
 */
template <class StateT, class ControlT>
class VisualServoingReferenceConnector
    : public ControllerConnector<std::tuple<tf::Transform, tf::Transform>,
                                 PositionYaw,
                                 ReferenceTrajectoryPtr<StateT, ControlT>>,
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
  VisualServoingReferenceConnector(
      BaseTracker &tracker, parsernode::Parser &drone_hardware,
      GenerateReferenceTrajectoryController &controller,
      tf::Transform camera_transform,
      tf::Transform tracking_offset_transform = tf::Transform::getIdentity())
      : ControllerConnector(controller, ControllerGroup::HighLevel),
        BaseRelativePoseVisualServoingConnector(tracker, drone_hardware,
                                                camera_transform,
                                                tracking_offset_transform),
  {
    DATA_HEADER("rpyt_relative_pose_visual_servoing_connector")
        << "vel_x"
        << "vel_y"
        << "vel_z"
        << "roll"
        << "pitch"
        << "yaw"
        << "omega_x"
        << "omega_y"
        << "omega_z"
        << "tracking_x"
        << "tracking_y"
        << "tracking_z"
        << "tracking_r"
        << "tracking_p"
        << "tracking_y"
        << "Viewing_angle"
        << "Tracking_length" << DataStream::endl;
  }
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingReferenceConnector() {}
  /**
   * @brief set goal to controller and clear estimator buffer
   *
   * @param goal empty goal
   */
  void setGoal(PositionYaw goal);
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
   * @brief Extracts pose data from tracker
   *
   * @param sensor_data Current transform of quadrotor in the
   * rotation-compensated frame of the quadrotor; tracking transform in the
   * rotation-compensated frame of the quadrotor; current velocity and
   * yawrate of the UAV
   *
   * @return true if able to compute transforms
   */
  virtual bool
  extractSensorData(std::tuple<tf::Transform, tf::Transform> &sensor_data);

  /**
   * @brief Send velocity commands to hardware
   *
   * @param controls roll, pitch, yawrate, thrust to send to UAV
   */
  virtual void
  sendControllerCommands(ReferenceTrajectoryPtr<StateT, ControlT> controls);

private:
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass =
      ControllerConnector<std::tuple<tf::Transform, tf::Transform>, PositionYaw,
                          ReferenceTrajectoryPtr<StateT, ControlT>>;
  /**
   * @brief Estimator for finding the gain between joystick thrust command and
   * the acceleration in body z direction
   */
  ThrustGainEstimator &thrust_gain_estimator_;
};
