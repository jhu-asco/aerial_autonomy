#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"
#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/minimum_snap_reference_trajectory.h"

#include <gcop/hrotor.h>
#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

using namespace quad_simulator;
using namespace gcop;
using namespace test_utils;

class QrotorBacksteppingControllerConnectorTests : public ::testing::Test {
public:
  /* Todo Constructor
  */
  QrotorBacksteppingControllerConnectorTests()
      : thrust_gain_estimator_(0.16), sys() {
    config_.set_mass(sys.m);
    config_.set_jxx(sys.J(0));
    config_.set_jyy(sys.J(1));
    config_.set_jzz(sys.J(2));
    config_.set_k2(0.2);
    config_.set_k1(0.2);
    config_.set_kp_xy(2.5);
    config_.set_kp_z(2.5);
    config_.set_kd_xy(3.3);
    config_.set_kd_z(3.3);
    auto vel_tolerance = config_.mutable_goal_velocity_tolerance();
    vel_tolerance->set_vx(0.0005);
    vel_tolerance->set_vy(0.0005);
    vel_tolerance->set_vz(0.0005);

    auto pos_tolerance = config_.mutable_goal_position_tolerance();
    pos_tolerance->set_x(0.00005);
    pos_tolerance->set_y(0.00005);
    pos_tolerance->set_z(0.00005);
    controller_.reset(new QrotorBacksteppingController(config_));
    controller_connector_.reset(new QrotorBacksteppingControllerConnector(
        drone_hardware_, *controller_, thrust_gain_estimator_, config_,
        std::chrono::duration<double>(0.02)));
    drone_hardware_.usePerfectTime();
  }

  /* Todo static void SetUpTestCase()
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
  }

  void runUntilConvergence(
      std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal,
      bool check_thrust_gain = true) {
    ParticleState initial_desired_state = std::get<0>(goal->atTime(0.0));
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_desired_state.p.x + .5;
    init_position.y = initial_desired_state.p.y + .5;
    init_position.z = initial_desired_state.p.z + .5;
    drone_hardware_.cmdwaypoint(init_position);
    controller_connector_->setGoal(goal);
    auto runController = [&]() {
      controller_connector_->run();
      return controller_connector_->getStatus() == ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(runController,
                                              std::chrono::seconds(60),
                                              std::chrono::milliseconds(20)));
    // Check position is goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    /* Homogenous mat for current quad */
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    /* Homogenous mat for Goal */
    ParticleState goal_desired_state = std::get<0>(goal->atTime(1));
    tf::Transform goal_transform(tf::createQuaternionFromRPY(0, 0, 0),
                                 tf::Vector3(goal_desired_state.p.x,
                                             goal_desired_state.p.y,
                                             goal_desired_state.p.z));

    ASSERT_TF_NEAR(quad_transform, goal_transform, 0.1);
    ASSERT_EQ(controller_connector_->getStatus(), ControllerStatus::Completed);
  }
  ThrustGainEstimator thrust_gain_estimator_;
  Hrotor sys;
  QrotorBacksteppingControllerConfig config_;
  QuadSimulator drone_hardware_;
  std::unique_ptr<QrotorBacksteppingController> controller_;
  std::unique_ptr<QrotorBacksteppingControllerConnector> controller_connector_;
};

TEST_F(QrotorBacksteppingControllerConnectorTests, test1) {
  int r = 4;
  Eigen::VectorXd tau_vec(1);
  tau_vec << 60;
  Eigen::MatrixXd path(2, 3);
  path << 0, 0, 0, 1, 1, 1;

  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  runUntilConvergence(goal);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
