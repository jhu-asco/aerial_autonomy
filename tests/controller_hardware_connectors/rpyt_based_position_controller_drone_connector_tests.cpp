#include "aerial_autonomy/controller_hardware_connectors/rpyt_based_position_controller_drone_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_position_controller.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

using namespace quad_simulator;

using namespace test_utils;

class RPYTBasedPositionControllerDroneConnectorTests : public ::testing::Test {
public:
  RPYTBasedPositionControllerDroneConnectorTests()
      : goal_tolerance_position_(0.05), goal_tolerance_velocity_(0.05),
        goal_tolerance_yaw_(0.05), goal_tolerance_yaw_rate_(0.05),
        thrust_gain_estimator_(0.18) {
    RPYTBasedPositionControllerConfig config;
    auto position_controller_config =
        config.mutable_velocity_based_position_controller_config();
    position_controller_config->set_position_gain(1.0);
    position_controller_config->set_max_velocity(1.0);
    position_controller_config->set_yaw_gain(1.0);
    position_controller_config->set_max_yaw_rate(1.0);
    position_controller_config->set_yaw_i_gain(0.0);
    position_controller_config->set_position_i_gain(0.0);
    position_controller_config->set_position_saturation_value(0.0);
    position_controller_config->set_yaw_saturation_value(0.0);
    auto position_tolerance =
        position_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    position_tolerance->set_x(goal_tolerance_position_);
    position_tolerance->set_y(goal_tolerance_position_);
    position_tolerance->set_z(goal_tolerance_position_);
    position_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(goal_tolerance_yaw_);
    auto velocity_controller_config =
        config.mutable_rpyt_based_velocity_controller_config();
    auto velocity_tolerance =
        velocity_controller_config->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    velocity_tolerance->set_vx(goal_tolerance_velocity_);
    velocity_tolerance->set_vy(goal_tolerance_velocity_);
    velocity_tolerance->set_vz(goal_tolerance_velocity_);
    velocity_controller_config->mutable_velocity_controller_config()
        ->set_goal_yaw_rate_tolerance(goal_tolerance_yaw_rate_);
    controller_.reset(
        new RPYTBasedPositionController(config, std::chrono::milliseconds(20)));
    controller_connector_.reset(new RPYTBasedPositionControllerDroneConnector(
        drone_hardware_, *controller_, thrust_gain_estimator_));
    drone_hardware_.usePerfectTime();
  }

  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("velocity_based_position_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("rpyt_based_velocity_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
  }

  void runUntilConvergence(const PositionYaw &initial_state,
                           const PositionYaw &goal,
                           bool check_thrust_gain = true) {
    // Fly quadrotor which sets the altitude to 0.5
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_state.x;
    init_position.y = initial_state.y;
    init_position.z = initial_state.z;
    drone_hardware_.cmdwaypoint(init_position, initial_state.yaw);
    controller_connector_->setGoal(goal);
    auto runController = [&]() {
      controller_connector_->run();
      return controller_connector_->getStatus() == ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(
        runController, std::chrono::seconds(1), std::chrono::milliseconds(0)));
    // Check position is goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    tf::Transform goal_transform(tf::createQuaternionFromRPY(0, 0, goal.yaw),
                                 tf::Vector3(goal.x, goal.y, goal.z));
    ASSERT_TF_NEAR(quad_transform, goal_transform, goal_tolerance_yaw_);
    ASSERT_EQ(controller_connector_->getStatus(), ControllerStatus::Completed);
    if (check_thrust_gain) {
      ASSERT_NEAR(thrust_gain_estimator_.getThrustGain(), 0.16, 1e-4);
    }
  }

  QuadSimulator drone_hardware_;
  std::unique_ptr<RPYTBasedPositionController> controller_;
  std::unique_ptr<RPYTBasedPositionControllerDroneConnector>
      controller_connector_;

  double goal_tolerance_position_;
  double goal_tolerance_velocity_;
  double goal_tolerance_yaw_;
  double goal_tolerance_yaw_rate_;

  ThrustGainEstimator thrust_gain_estimator_;
};

TEST_F(RPYTBasedPositionControllerDroneConnectorTests, GoalIsStart) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(0, 0, 0, 0), false);
}

TEST_F(RPYTBasedPositionControllerDroneConnectorTests,
       ConvergenceZeroStartNoYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(1, -1, 1, 0));
}

TEST_F(RPYTBasedPositionControllerDroneConnectorTests,
       ConvergenceZeroStartPosYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(-2, -3, 3, 0.5));
}

TEST_F(RPYTBasedPositionControllerDroneConnectorTests,
       ConvergenceZeroStartNegYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(1, -1, -1, -0.5));
}

TEST_F(RPYTBasedPositionControllerDroneConnectorTests,
       ConvergenceNonZeroStartNoYaw) {
  runUntilConvergence(PositionYaw(-1, 3.3, 2, 0), PositionYaw(1, -1, 1, 0));
}

TEST_F(RPYTBasedPositionControllerDroneConnectorTests,
       ConvergenceNonZeroStartPosYaw) {
  runUntilConvergence(PositionYaw(-2.2, -1, 3.2, -0.1),
                      PositionYaw(-2, -3, 3, 0.5));
}

TEST_F(RPYTBasedPositionControllerDroneConnectorTests,
       ConvergenceNonZeroStartNegYaw) {
  runUntilConvergence(PositionYaw(-.1, 3.2, 1, 0.4),
                      PositionYaw(1, -1, -1, -0.5));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
