#pragma once
#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"
#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/sensors/flight_control_sensor.h"
#include "aerial_autonomy/sensors/pose_sensor.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/minimum_snap_reference_trajectory.h"
#include <glog/logging.h>
#include <quad_simulator_parser/quad_simulator.h>

using namespace quad_simulator;
using namespace gcop;
using namespace test_utils;

class QrotorBacksteppingControllerConnectorTests : public ::testing::Test {
public:
  /**
  * @brief Constructor
  */
  QrotorBacksteppingControllerConnectorTests() : thrust_gain_estimator_(0.16) {
    config_.set_mass(3.4);
    config_.set_jxx(0.05);
    config_.set_jyy(0.05);
    config_.set_jzz(0.08);
    config_.set_k2(0.35);
    config_.set_k1(0.35);
    config_.set_kp_xy(40);
    config_.set_kp_z(40);
    config_.set_kd_xy(40);
    config_.set_kd_z(40);
    auto pos_tolerance = config_.mutable_goal_position_tolerance();
    pos_tolerance->set_x(0.02);
    pos_tolerance->set_y(0.02);
    pos_tolerance->set_z(0.02);
    auto vel_tolerance = config_.mutable_goal_velocity_tolerance();
    vel_tolerance->set_vx(0.02);
    vel_tolerance->set_vy(0.02);
    vel_tolerance->set_vz(0.02);
    odom_sensor_.reset(new FlightControlSensor(drone_hardware_));
    controller_.reset(new QrotorBacksteppingController(config_));

    drone_hardware_.usePerfectTime();
  }
  /**
  * @brief Configure logger
  */
  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("qrotor_backstepping_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("qrotor_backstepping_controller_connector");
    Log::instance().addDataStream(data_config);
  }
  /**
  * @brief run the backstepping controller until its either converged/crashed
  *
  * @param goal reference trajectory
  * @param pos_err Initial position error
  * @param vel_err Initial velocity error
  */
  void runUntilConvergence(
      std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
      double total_time, tf::Vector3 pos_err, tf::Vector3 vel_err,
      bool check_thrust_gain = true, bool using_odom_sensor = false) {

    // Use pose sensor instead of quaddata
    if (!using_odom_sensor) {
      controller_connector_.reset(new QrotorBacksteppingControllerConnector(
          drone_hardware_, *controller_, thrust_gain_estimator_, config_,
          odom_sensor_));
    } else {
      // Use quaddata
      controller_connector_.reset(new QrotorBacksteppingControllerConnector(
          drone_hardware_, *controller_, thrust_gain_estimator_, config_));
    }
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();

    ParticleState initial_desired_state = std::get<0>(goal->atTime(0.0));

    // Initial position (with disturbances)
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_desired_state.p.x + pos_err[0];
    init_position.y = initial_desired_state.p.y + pos_err[1];
    init_position.z = initial_desired_state.p.z + pos_err[2];
    drone_hardware_.cmdwaypoint(init_position);

    // Initial velocity (with disturbances)
    geometry_msgs::Vector3 init_linvel;
    init_linvel.x = initial_desired_state.v.x + vel_err[0];
    init_linvel.y = initial_desired_state.v.y + vel_err[1];
    init_linvel.z = initial_desired_state.v.z + vel_err[2];
    double init_yaw = 0.0;
    drone_hardware_.cmdvel_yaw_angle_guided(init_linvel, init_yaw);

    controller_connector_->setGoal(goal);
    auto runController = [&]() {
      controller_connector_->run();
      return controller_connector_->getStatus() == ControllerStatus::Active;
    };

    ASSERT_FALSE(test_utils::waitUntilFalse()(runController,
                                              std::chrono::seconds(60),
                                              std::chrono::milliseconds(20)));

    // Check if the position is at goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    ParticleState goal_desired_state = std::get<0>(goal->atTime(total_time));
    tf::Transform goal_transform(tf::createQuaternionFromRPY(0, 0, 0),
                                 tf::Vector3(goal_desired_state.p.x,
                                             goal_desired_state.p.y,
                                             goal_desired_state.p.z));

    ASSERT_TF_NEAR(quad_transform, goal_transform, 0.1);
    ASSERT_EQ(controller_connector_->getStatus(), ControllerStatus::Completed);
  }

  /**
  * @brief run the backstepping controller until its either converged, or the
  * control input is out of bounds
  * @param goal reference trajectory
  * @param pos_err Initial position error
  * @param vel_err Initial velocity error
  */
  void runUntilOutOfBounds(
      std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
      double total_time, tf::Vector3 pos_err, tf::Vector3 vel_err,
      bool check_thrust_gain = true) {

    // Use quaddata
    controller_connector_.reset(new QrotorBacksteppingControllerConnector(
        drone_hardware_, *controller_, thrust_gain_estimator_, config_));
    ParticleState initial_desired_state = std::get<0>(goal->atTime(0.0));

    // Initial position (with disturbances)
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_desired_state.p.x + pos_err[0];
    init_position.y = initial_desired_state.p.y + pos_err[1];
    init_position.z = initial_desired_state.p.z + pos_err[2];
    drone_hardware_.cmdwaypoint(init_position);

    // Initial velocity (with disturbances)
    geometry_msgs::Vector3 init_linvel;
    init_linvel.x = initial_desired_state.v.x + vel_err[0];
    init_linvel.y = initial_desired_state.v.y + vel_err[1];
    init_linvel.z = initial_desired_state.v.z + vel_err[2];
    double init_yaw = 0.0;
    drone_hardware_.cmdvel_yaw_angle_guided(init_linvel, init_yaw);

    controller_connector_->setGoal(goal);
    while (controller_connector_->getStatus() != ControllerStatus::Completed) {
      controller_connector_->run();
      ASSERT_GE(controller_connector_->getLastCommand().x, -0.785);
      ASSERT_LE(controller_connector_->getLastCommand().x, 0.785);
      ASSERT_GE(controller_connector_->getLastCommand().y, -0.785);
      ASSERT_LE(controller_connector_->getLastCommand().y, 0.785);
      ASSERT_GE(controller_connector_->getLastCommand().z, -1.5708);
      ASSERT_LE(controller_connector_->getLastCommand().z, 1.5708);
      ASSERT_GE(controller_connector_->getLastCommand().w *
                    thrust_gain_estimator_.getThrustGain(),
                (0.8 - 1e-5) * config_.acc_gravity());
      ASSERT_LE(controller_connector_->getLastCommand().w *
                    thrust_gain_estimator_.getThrustGain(),
                (1.2 + 1e-5) * config_.acc_gravity());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

private:
  QuadSimulator drone_hardware_;
  // std::shared_ptr<Sensor<tf::StampedTransform>> pose_sensor_;
  std::shared_ptr<Sensor<std::pair<tf::StampedTransform, tf::Vector3>>>
      odom_sensor_;
  std::unique_ptr<QrotorBacksteppingController> controller_;
  std::unique_ptr<QrotorBacksteppingControllerConnector> controller_connector_;
  ThrustGainEstimator thrust_gain_estimator_;
  QrotorBacksteppingControllerConfig config_;
};
