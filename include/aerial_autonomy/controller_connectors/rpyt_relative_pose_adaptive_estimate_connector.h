#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_relative_pose_adaptive_estimate_controller.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust_adaptive.h"
#include "aerial_autonomy/types/snap.h"
#include "rpyt_based_relative_pose_adaptive_estimate_controller_config.pb.h"
#include <chrono>
#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A connector for using an adaptive estimator and RPYT controls
 * to track a given reference trajectory
 */
class RPYTRelativePoseAdaptiveEstimateConnector
    : public ControllerConnector<
          std::tuple<double, double, ParticleState>,
          std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double>,
          RollPitchYawThrustAdaptive> {
public:
  /**
   * @brief Constructor
   * @param drone_hardware UAV hardware to send controller commands
   * @param controller Controller to compute control inputs
   */
  RPYTRelativePoseAdaptiveEstimateConnector(
      parsernode::Parser &drone_hardware,
      RPYTBasedRelativePoseAdaptiveEstimateController &controller,
      double mhat_initial, double min_m = 0.5,
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor =
          nullptr)
      : ControllerConnector(controller, ControllerGroup::UAV),
        private_reference_controller_(controller), odom_sensor_(odom_sensor),
        drone_hardware_(drone_hardware) {
    previous_time_ = std::chrono::high_resolution_clock::now();
    mhat = mhat_initial;
    min_m_ = min_m;
  }
  /**
   * @brief Destructor
   */
  virtual ~RPYTRelativePoseAdaptiveEstimateConnector() {}
  /**
   * @brief set goal to controller
   *
   */
  void
  setGoal(std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double> goal);

protected:
  /**
   * @brief Extracts sensor data (time and particle state)
   *
   * @param sensor_data mhat and particle state data.
   *
   * @return true if able to compute transforms
   */
  virtual bool
  extractSensorData(std::tuple<double, double, ParticleState> &sensor_data);

  /**
   * @brief Send velocity commands to hardware
   *
   * @param controls roll, pitch, yawrate, thrust to send to UAV
   */
  virtual void sendControllerCommands(RollPitchYawThrustAdaptive controls);

private:
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass = ControllerConnector<
      std::tuple<double, double, ParticleState>,
      std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double>,
      RollPitchYawThrustAdaptive>;
  /**
  * @brief Internal reference to controller that is connected by this class
  */
  RPYTBasedRelativePoseAdaptiveEstimateController
      &private_reference_controller_;
  /**
  * @brief Internal estimate of mass parameter mhat
  */
  double mhat;
  double min_m_;
  /**
  * @brief Time when the last sendControllerCommands is called
  */
  std::chrono::time_point<std::chrono::high_resolution_clock> previous_time_;
  /**
   * @brief Pose sensor for quad data
   */
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_;
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
