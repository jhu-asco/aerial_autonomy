#pragma once
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controller_connectors/base_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_relative_pose_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and brings the quadrotor
 * to a goal pose expressed in the tracked object's coordinate frame
 */
class RPYTRelativePoseVisualServoingConnector
    : public ControllerConnector<
          std::tuple<tf::Transform, tf::Transform, VelocityYawRate>,
          PositionYaw, RollPitchYawRateThrust>,
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
  RPYTRelativePoseVisualServoingConnector(
      BaseTracker &tracker, parsernode::Parser &drone_hardware,
      RPYTBasedRelativePoseController &controller,
      ThrustGainEstimator &thrust_gain_estimator,
      tf::Transform camera_transform,
      tf::Transform tracking_offset_transform = tf::Transform::getIdentity())
      : ControllerConnector(controller, ControllerGroup::UAV),
        BaseRelativePoseVisualServoingConnector(tracker, drone_hardware,
                                                camera_transform,
                                                tracking_offset_transform),
        thrust_gain_estimator_(thrust_gain_estimator),
        private_reference_controller_(controller) {
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
  virtual ~RPYTRelativePoseVisualServoingConnector() {}
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
  virtual bool extractSensorData(
      std::tuple<tf::Transform, tf::Transform, VelocityYawRate> &sensor_data);

  /**
   * @brief Send velocity commands to hardware
   *
   * @param controls roll, pitch, yawrate, thrust to send to UAV
   */
  virtual void sendControllerCommands(RollPitchYawRateThrust controls);

private:
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass = ControllerConnector<
      std::tuple<tf::Transform, tf::Transform, VelocityYawRate>, PositionYaw,
      RollPitchYawRateThrust>;
  /**
   * @brief Estimator for finding the gain between joystick thrust command and
   * the acceleration in body z direction
   */
  ThrustGainEstimator &thrust_gain_estimator_;
  /**
   * @brief Internal reference to controller that is connected by this class
   */
  RPYTBasedRelativePoseController &private_reference_controller_;
};
