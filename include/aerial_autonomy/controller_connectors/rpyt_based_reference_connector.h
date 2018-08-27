#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_reference_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
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
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor =
          nullptr)
      : BaseClass(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware), odom_sensor_(odom_sensor),
        thrust_gain_estimator_(thrust_gain_estimator),
        t_init_(std::chrono::high_resolution_clock::now()), time_since_init_(0),
        use_perfect_time_diff_(config.use_perfect_time_diff()),
        perfect_time_diff_(config.perfect_time_diff()) {
    DATA_HEADER("rpyt_reference_connector") << "Thrust_gain"
                                            << "x"
                                            << "y"
                                            << "z"
                                            << "roll"
                                            << "pitch"
                                            << "yaw"
                                            << "vx"
                                            << "vy"
                                            << "vz"
                                            << "roll_bias"
                                            << "pitch_bias"
                                            << "Sensor_roll"
                                            << "Sensor_pitch"
                                            << "Sensor_yaw" << DataStream::endl;
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

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
   * @brief Pose sensor for quad data
   */
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_;
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
};

template <class StateT, class ControlT>
bool RPYTBasedReferenceConnector<StateT, ControlT>::extractSensorData(
    std::tuple<double, double, Velocity, PositionYaw> &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  PositionYaw position_yaw;
  Velocity velocity;
  double sensor_r = 0, sensor_p = 0, sensor_y = 0;
  if (odom_sensor_) {
    if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Sensor invalid";
      return false;
    }
    auto quad_pose_velocity_pair = odom_sensor_->getSensorData();
    conversions::tfToPositionYaw(position_yaw, quad_pose_velocity_pair.first);
    quad_pose_velocity_pair.first.getBasis().getRPY(sensor_r, sensor_p,
                                                    sensor_y);
    // Get Velocity
    tf::Vector3 &sensor_velocity = quad_pose_velocity_pair.second;
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
  tf::Vector3 body_acc(data.linacc.x, data.linacc.y, data.linacc.z);
  thrust_gain_estimator_.addSensorData(data.rpydata.x, data.rpydata.y,
                                       body_acc);
  Eigen::Vector2d roll_pitch_bias = thrust_gain_estimator_.getRollPitchBias();
  DATA_LOG("rpyt_reference_connector")
      << std::get<1>(sensor_data) << position_yaw.x << position_yaw.y
      << position_yaw.z << data.rpydata.x << data.rpydata.y << position_yaw.yaw
      << velocity.x << velocity.y << velocity.z << roll_pitch_bias << sensor_r
      << sensor_p << sensor_y << DataStream::endl;
  return true;
}

template <class StateT, class ControlT>
void RPYTBasedReferenceConnector<StateT, ControlT>::sendControllerCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  // Eigen::Vector2d roll_pitch_bias =
  // thrust_gain_estimator_.getRollPitchBias();
  // rpyt_msg.x = controls.r - roll_pitch_bias[0];
  // rpyt_msg.y = controls.p - roll_pitch_bias[1];
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
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}
