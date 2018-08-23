#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_reference_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include "rpyt_reference_connector_config.pb.h"

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
      ThrustGainEstimator &thrust_gain_estimator,
      RPYTReferenceConnectorConfig config,
      SensorPtr<tf::StampedTransform> pose_sensor = nullptr)
      : BaseClass(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware), pose_sensor_(pose_sensor),
        thrust_gain_estimator_(thrust_gain_estimator),
        t_init_(std::chrono::high_resolution_clock::now()),
        previous_measurement_time_(std::chrono::high_resolution_clock::now()),
        time_since_init_(0),
        use_perfect_time_diff_(config.use_perfect_time_diff()),
        perfect_time_diff_(config.perfect_time_diff()),
        previous_measurement_initialized_(false),
        previous_position_(tf::Vector3(0, 0, 0)),
        velocity_filter_(config.velocity_exp_gain()) {}
  /**
   * @brief set goal to controller and clear estimator buffer
   *
   * @param goal empty goal
   */
  void setGoal(ReferenceTrajectoryPtr<StateT, ControlT> goal) {
    BaseClass::setGoal(goal);
    VLOG(1) << "Clearing thrust estimator buffer";
    thrust_gain_estimator_.clearBuffer();
  }

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
   * @brief Time since initialization
   */
  double time_since_init_;
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

template <class StateT, class ControlT>
bool RPYTBasedReferenceConnector<StateT, ControlT>::extractSensorData(
    std::tuple<double, double, Velocity, PositionYaw> &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  PositionYaw position_yaw;
  Velocity velocity;
  if (pose_sensor_) {
    if (pose_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Sensor invalid";
      return false;
    }
    tf::Transform quad_pose = pose_sensor_->getSensorData();
    conversions::tfToPositionYaw(position_yaw, quad_pose);
    // Get Velocity
    tf::Vector3 sensor_velocity = getVelocity(quad_pose.getOrigin());
    velocity =
        Velocity(sensor_velocity.x(), sensor_velocity.y(), sensor_velocity.z());
  } else {
    position_yaw = PositionYaw(data.localpos.x, data.localpos.y,
                               data.localpos.z, data.rpydata.z);
    velocity = Velocity(data.linvel.x, data.linvel.y, data.linvel.z);
  }
  if (use_perfect_time_diff_) {
    time_since_init_ += perfect_time_diff_;
  } else {
    time_since_init_ = std::chrono::duration<double>(
                           std::chrono::high_resolution_clock::now() - t_init_)
                           .count();
  }
  sensor_data =
      std::make_tuple(time_since_init_, thrust_gain_estimator_.getThrustGain(),
                      velocity, position_yaw);
  thrust_gain_estimator_.addSensorData(data.rpydata.x, data.rpydata.y,
                                       data.linacc.z);
  return true;
}

template <class StateT, class ControlT>
tf::Vector3 RPYTBasedReferenceConnector<StateT, ControlT>::getVelocity(
    tf::Vector3 current_position) {
  double dt = getTimeDiff();
  tf::Vector3 velocity(0, 0, 0);
  if (previous_measurement_initialized_) {
    velocity = (current_position - previous_position_) / dt;
    previous_measurement_initialized_ = false;
  }
  return velocity;
}

template <class StateT, class ControlT>
double RPYTBasedReferenceConnector<StateT, ControlT>::getTimeDiff() {
  // Timing logic
  auto current_time = std::chrono::high_resolution_clock::now();
  double dt =
      std::chrono::duration<double>(current_time - previous_measurement_time_)
          .count();
  if (use_perfect_time_diff_) {
    dt = perfect_time_diff_;
  }
  previous_measurement_time_ = current_time;
  if (dt < 1e-4) {
    LOG(WARNING) << "Time diff cannot be smaller than 1e-4";
    dt = 1e-4;
  }
  return dt;
}

template <class StateT, class ControlT>
void RPYTBasedReferenceConnector<StateT, ControlT>::sendControllerCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

template <class StateT, class ControlT>
void RPYTBasedReferenceConnector<StateT, ControlT>::initialize() {
  t_init_ = std::chrono::high_resolution_clock::now();
  time_since_init_ = 0.0;
  velocity_filter_.reset();
  previous_measurement_initialized_ = false;
}
