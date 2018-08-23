#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_reference_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <chrono>
#include <parsernode/parser.h>
/**
 * @brief Manages communication between a UAV plugin and a position controller
 * that outputs a rpyt command
 */
template <class StateT, class ControlT>
class RPYTBasedReferenceConnector
    : public ControllerConnector<
          std::tuple<double, double, Velocity, PositionYaw>,
          ReferenceTrajectoryPtr<StateT, ControlT>, RollPitchYawRateThrust> {

  /**
  * @brief  Base class
  */
  using BaseClass =
      ControllerConnector<std::tuple<double, double, Velocity, PositionYaw>,
                          ReferenceTrajectoryPtr<StateT, ControlT>,
                          RollPitchYawRateThrust>;

public:
  /**
  * @brief Constructor
  * @param drone_hardware Plugin used to send commands to UAV
  * @param controller Position controller that gives rpyt commands
  * @param thrust_gain_estimator Thrust gain estimator
  * \todo add config
  */
  RPYTBasedReferenceConnector(
      parsernode::Parser &drone_hardware,
      AbstractRPYTBasedReferenceController<StateT, ControlT> &controller,
      ThrustGainEstimator &thrust_gain_estimator, double exp_gain = 1.0,
      bool use_perfect_time_diff = false,
      SensorPtr<tf::StampedTransform> pose_sensor = nullptr)
      : BaseClass(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware), pose_sensor_(pose_sensor),
        thrust_gain_estimator_(thrust_gain_estimator),
        t_init_(std::chrono::high_resolution_clock::now()),
        previous_measurement_time_(std::chrono::high_resolution_clock::now()),
        use_perfect_time_diff_(use_perfect_time_diff), perfect_time_diff_(0.02),
        previous_measurement_initialized_(false),
        previous_position_(tf::Vector3(0, 0, 0)), velocity_filter_(exp_gain) {}
  /**
   * @brief set goal to controller and clear estimator buffer
   *
   * @param goal empty goal
   */
  void setGoal(PositionYaw goal);

protected:
  /**
   * @brief extracts position and velocity data from UAV to compute appropriate
   * rpyt
   * commands
   *
   * @param sensor_data Current position and velocity of UAV
   *
   * @return true if sensor data can be extracted
   */
  virtual bool extractSensorData(
      std::tuple<double, double, Velocity, PositionYaw> &sensor_data);

  /**
   * @brief  Send rpyt commands to hardware
   *
   * @param controls rpyt commands to send to UAV
   */
  virtual void sendControllerCommands(RollPitchYawRateThrust controls);

  /**
   * @brief Initialize connector
   */
  void initialize();

  /**
   * @brief Get time difference
   *
   * @return time difference
   */
  double getTimeDiff();
  /**
   * @brief Get velocity
   *
   * @return get velocity by finite diff
   */
  tf::Vector3 getVelocity(tf::Vector3 current_position);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
   * @brief Pose sensor for quad data
   */
  SensorPtr<tf::StampedTransform> pose_sensor_;
  /**
   * @brief Estimator for finding the gain between joystick thrust command and
   * the acceleration in body z direction
   */
  ThrustGainEstimator &thrust_gain_estimator_;
  /**
   * @brief Time when connector is initialized
   */
  std::chrono::high_resolution_clock::time_point t_init_;
  /**
   * @brief Previous measurment time
   */
  std::chrono::high_resolution_clock::time_point previous_measurement_time_;
  /**
   * @brief Use perfect time diff for finite diff
   */
  bool use_perfect_time_diff_;
  /**
   * @brief Perfect time diff to use for finite diff
   */
  const double perfect_time_diff_;
  /**
   * @brief Previous measurement initialized flag
   */
  bool previous_measurement_initialized_;
  /**
   * @brief Previous position measurement
   */
  tf::Vector3 previous_position_;
  /**
   * @brief Filter for velocity
   */
  ExponentialFilter<Eigen::Vector3d> velocity_filter_;
};
