#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controller_hardware_connectors/base_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_relative_pose_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/estimators/tracking_vector_estimator.h"
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
    : public ControllerHardwareConnector<
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
      TrackingVectorEstimator &tracking_vector_estimator,
      tf::Transform camera_transform,
      tf::Transform tracking_offset_transform = tf::Transform::getIdentity())
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        BaseRelativePoseVisualServoingConnector(tracker, drone_hardware,
                                                camera_transform,
                                                tracking_offset_transform),
        thrust_gain_estimator_(thrust_gain_estimator),
        tracking_vector_estimator_(tracking_vector_estimator),
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
        << "tracking_y" << DataStream::endl;
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
  virtual void sendHardwareCommands(RollPitchYawRateThrust controls);

private:
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass = ControllerHardwareConnector<
      std::tuple<tf::Transform, tf::Transform, VelocityYawRate>, PositionYaw,
      RollPitchYawRateThrust>;
  ThrustGainEstimator &thrust_gain_estimator_;
  TrackingVectorEstimator &tracking_vector_estimator_;
  RPYTBasedRelativePoseController &private_reference_controller_;
};
