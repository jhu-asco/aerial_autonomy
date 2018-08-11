#pragma once
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controller_connectors/base_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"

#include <tf/tf.h>
#include <utility> // Pair

#include <parsernode/parser.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and brings the quadrotor
 * to a goal pose expressed in the tracked object's coordinate frame
 */
template <class StateT, class ControlT, class DependentConnectorT>
class VisualServoingReferenceConnector
    : public ControllerConnector<std::pair<PositionYaw, tf::Transform>,
                                 PositionYaw,
                                 ReferenceTrajectoryPtr<StateT, ControlT>>,
      public BaseRelativePoseVisualServoingConnector {
private:
  /**
   * @brief Controller that generates reference trajectory
   */
  using ReferenceGenerator =
      Controller<std::pair<PositionYaw, tf::Transform>, PositionYaw,
                 ReferenceTrajectoryPtr<StateT, ControlT>>;

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
      ReferenceGenerator &controller, DependentConnectorT &dependent_connector,
      tf::Transform camera_transform,
      tf::Transform tracking_offset_transform = tf::Transform::getIdentity())
      : BaseClass(controller, ControllerGroup::HighLevel),
        BaseRelativePoseVisualServoingConnector(tracker, drone_hardware,
                                                camera_transform,
                                                tracking_offset_transform),
        dependent_connector_(dependent_connector),
        start_position_yaw_(0, 0, 0, 0) {
    logTrackerHeader("visual_servoing_reference_connector");
  }

  void initialize() {
    parsernode::common::quaddata quad_data;
    drone_hardware_.getquaddata(quad_data);
    start_position_yaw_ =
        PositionYaw(quad_data.localpos.x, quad_data.localpos.y,
                    quad_data.localpos.z, quad_data.rpydata.z);
    this->run(); // sets goal to low-level connector
  }
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingReferenceConnector() {}

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
  extractSensorData(std::pair<PositionYaw, tf::Transform> &sensor_data) {
    parsernode::common::quaddata quad_data;
    drone_hardware_.getquaddata(quad_data);
    tf::Transform object_pose_cam;
    if (!tracker_.getTrackingVector(object_pose_cam)) {
      VLOG(1) << "Invalid tracking vector";
      return false;
    }
    tf::Vector3 quad_origin(quad_data.localpos.x, quad_data.localpos.y,
                            quad_data.localpos.z);
    tf::Transform tracking_pose =
        getTrackingTransformRotationCompensatedQuadFrame(object_pose_cam);
    logTrackerData("visual_servoing_reference_connector", tracking_pose,
                   object_pose_cam, quad_data);
    tracking_pose.setOrigin(quad_origin + tracking_pose.getOrigin());
    sensor_data = std::make_pair(start_position_yaw_, tracking_pose);
    return true;
  }

  /**
   * @brief Send velocity commands to hardware
   *
   * @param controls roll, pitch, yawrate, thrust to send to UAV
   */
  virtual void
  sendControllerCommands(ReferenceTrajectoryPtr<StateT, ControlT> control) {
    dependent_connector_.setGoal(control);
  }

  AbstractControllerConnector *getDependentConnector() {
    return &dependent_connector_;
  }

private:
  DependentConnectorT &dependent_connector_;
  PositionYaw start_position_yaw_;
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass =
      ControllerConnector<std::pair<PositionYaw, tf::Transform>, PositionYaw,
                          ReferenceTrajectoryPtr<StateT, ControlT>>;
};
