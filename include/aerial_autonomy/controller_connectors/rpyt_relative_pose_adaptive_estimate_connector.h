#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_relative_pose_adaptive_estimate_controller.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/particle_state_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust_adaptive.h"
#include "aerial_autonomy/types/snap.h"
#include "rpyt_based_relative_pose_adaptive_estimate_controller_config.pb.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include <chrono>
#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A connector for using an adaptive estimator and RPYT controls
 * to track a given reference trajectory
 */
class RPYTRelativePoseAdaptiveEstimateConnector
    : public ControllerConnector<
          std::tuple<double, double, ParticleStateYaw>,
          ReferenceTrajectoryPtr<ParticleStateYaw, Snap>,
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
      ThrustGainEstimator &thrust_gain_estimator,
      double mhat_initial, double min_m = 0.5,
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor =
          nullptr, bool use_perfect_time_diff = false, double perfect_time_diff = 0.02)
      : ControllerConnector(controller, ControllerGroup::UAV),
        private_reference_controller_(controller), odom_sensor_(odom_sensor),
        drone_hardware_(drone_hardware),
        thrust_gain_estimator_(thrust_gain_estimator),
        use_perfect_time_diff_(use_perfect_time_diff),
        perfect_time_diff_(perfect_time_diff),
        time_since_init_(0){
    t_init_ = std::chrono::high_resolution_clock::now();
    previous_time_ = std::chrono::high_resolution_clock::now();
    mhat_ = mhat_initial;
    min_m_ = min_m;
    if (mhat_ < min_m_) {
      mhat_ = min_m_;
    }
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
  setGoal(ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal);

protected:
  /**
   * @brief Extracts sensor data (time and particle state)
   *
   * @param sensor_data mhat and particle state data.
   *
   * @return true if able to compute transforms
   */
  virtual bool
  extractSensorData(std::tuple<double, double, ParticleStateYaw> &sensor_data);

  /**
   * @brief Send velocity commands to hardware
   *
   * @param controls roll, pitch, yawrate, thrust to send to UAV
   */
  virtual void sendControllerCommands(RollPitchYawThrustAdaptive controls);
  
  void initialize();

private:
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass = ControllerConnector<
      std::tuple<double, double, ParticleStateYaw>,
      ReferenceTrajectoryPtr<ParticleStateYaw, Snap>,
      RollPitchYawThrustAdaptive>;
  /**
  * @brief Internal reference to controller that is connected by this class
  */
  RPYTBasedRelativePoseAdaptiveEstimateController
      &private_reference_controller_;
  /**
  * @brief Internal estimate of mass parameter mhat
  */
  double mhat_;
  /**
  * @brief Minimum allowable m_hat
  */
  double min_m_;
  /**
  * @brief Time when the last initialize was called
  */
  std::chrono::high_resolution_clock::time_point t_init_;
  /**
  * @brief Time when the last sendControllerCommands was called
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
  //For comparison
  ThrustGainEstimator &thrust_gain_estimator_;
  bool use_perfect_time_diff_;
  const double perfect_time_diff_;
  double time_since_init_;
};
