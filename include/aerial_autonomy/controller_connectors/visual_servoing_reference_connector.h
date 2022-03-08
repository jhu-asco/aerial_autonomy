#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controller_connectors/base_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"

#include <tf/tf.h>
#include <utility> // Pair

#include <parsernode/parser.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and brings the quadrotor
 * to a goal reference expressed in the tracked object's coordinate frame
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
      tf::Transform tracking_offset_transform = tf::Transform::getIdentity(),
      double filter_gain_tracking_pose = 0.1,
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor =
          nullptr)
      : BaseClass(controller, ControllerGroup::HighLevel),
        BaseRelativePoseVisualServoingConnector(tracker, drone_hardware,
                                                camera_transform,
                                                tracking_offset_transform),
        dependent_connector_(dependent_connector),
        start_position_yaw_(0, 0, 0, 0), odom_sensor_(odom_sensor),
        tracking_pose_filter_(filter_gain_tracking_pose) {
    logTrackerHeader("visual_servoing_reference_connector");
  }

  void initialize() {
    BaseClass::initialize(); // Reset controller
    VLOG(1) << "initializing visual servoing reference";
    if (odom_sensor_) {
      if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
        LOG(WARNING)
            << "Pose sensor invalid! Cannot initialize visual servoing";
        return;
      }
      tf::StampedTransform pose = odom_sensor_->getSensorData().first;
      conversions::tfToPositionYaw(start_position_yaw_, pose);
      VLOG(2) << "Position yaw: " << start_position_yaw_.x << ", "
              << start_position_yaw_.y << ", " << start_position_yaw_.z << ", "
              << start_position_yaw_.yaw;
    } else {
      parsernode::common::quaddata quad_data;
      drone_hardware_.getquaddata(quad_data);
      start_position_yaw_ =
          PositionYaw(quad_data.localpos.x, quad_data.localpos.y,
                      quad_data.localpos.z, quad_data.rpydata.z);
    }
    tracking_pose_filter_.reset();
    this->run(); // sets goal to low-level connector
  }
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingReferenceConnector() {}

protected:
  /**
   * @brief Filter the position, yaw
   *
   * @param input pose
   *
   * @return filtered pose without roll, pitch
   */
  tf::Transform filter(tf::Transform input) {
    PositionYaw tracking_position_yaw;
    conversions::tfToPositionYaw(tracking_position_yaw, input);
    PositionYaw filtered_position_yaw =
        tracking_pose_filter_.addAndFilter(tracking_position_yaw);
    tf::Transform filtered_pose;
    conversions::positionYawToTf(filtered_position_yaw, filtered_pose);
    return filtered_pose;
  }
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
    tf::Transform quad_pose;
    if (odom_sensor_) {
      if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
        LOG(WARNING) << "Pose sensor invalid!";
        return false;
      }
      quad_pose = odom_sensor_->getSensorData().first;
    } else {
      quad_pose = conversions::getPose(quad_data);
    }

    ros::Time current_time = ros::Time::now();
    // If measurement is recent enough, recalculate tracking pose
    if ((current_time - tracker_.getROSTrackingTime()).toSec() < 0.1)
    { 
      tracking_pose_ = quad_pose * camera_transform_ *
                                    object_pose_cam * tracking_offset_transform_;
      // filter remove rp
      tracking_pose_ = filter(tracking_pose_);
      // Filter tracking pose
      logTrackerData("visual_servoing_reference_connector", tracking_pose_,
                    object_pose_cam, quad_data);
    }
    // Otherwise use previous tracking pose

    sensor_data = std::make_pair(start_position_yaw_, tracking_pose_);
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
  DependentConnectorT &dependent_connector_; ///< Dependent MPC Connector
  PositionYaw start_position_yaw_;           ///< Start position yaw
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>>
      odom_sensor_; ///< Pose sensor for quad data
  ExponentialFilter<PositionYaw>
      tracking_pose_filter_; ///< Filter tracking pose
  tf::Transform tracking_pose_; ///< Current estimate of tracking pose

  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass =
      ControllerConnector<std::pair<PositionYaw, tf::Transform>, PositionYaw,
                          ReferenceTrajectoryPtr<StateT, ControlT>>;
};
