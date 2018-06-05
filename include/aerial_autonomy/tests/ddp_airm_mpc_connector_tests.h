#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h"
#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/waypoint.h"

#include <Eigen/Dense>

#include "arm_parsers/arm_simulator.h"
#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

class MPCControllerAirmConnectorTests : public ::testing::Test {
public:
  MPCControllerAirmConnectorTests()
      : thrust_gain_estimator_(0.2), controller_config_(createConfig()) {
    controller_.reset(new DDPAirmMPCController(
        controller_config_, std::chrono::milliseconds(20), false));
    controller_connector_.reset(new MPCControllerAirmConnector(
        drone_hardware_, arm_simulator_, *controller_, thrust_gain_estimator_));
    drone_hardware_.usePerfectTime();
    drone_hardware_.set_delay_send_time(0.02);
    controller_connector_->usePerfectTimeDiff(0.02);
  }

  AirmMPCControllerConfig createConfig() {
    AirmMPCControllerConfig config;
    config.set_goal_position_tolerance(0.1);
    config.set_goal_velocity_tolerance(0.2);
    config.set_goal_joint_angle_tolerance(0.2);
    config.set_goal_joint_velocity_tolerance(0.1);
    DDPMPCControllerConfig *ddp_config = config.mutable_ddp_config();
    ddp_config->set_min_cost(500);
    ddp_config->set_look_ahead_time(0.02);
    auto q = ddp_config->mutable_q();
    q->Resize(21, 0.0);
    auto qf = ddp_config->mutable_qf();
    qf->Resize(21, 0.1);
    auto r = ddp_config->mutable_r();
    r->Resize(6, 0.1);
    r->Set(0, 2.0); // Reduce thrust control
    for (int i = 0; i < 3; ++i) {
      // Pos cost
      ddp_config->set_qf(i, 200.0);
      // Rpy cost
      ddp_config->set_qf(i + 3, 50.0);
      // Vel cost
      ddp_config->set_qf(i + 6, 10.0);
      ddp_config->set_q(i + 6, 1.0);
      // Rpydot cost
      ddp_config->set_qf(i + 9, 5.0);
    }
    for (int i = 0; i < 2; ++i) {
      // Joint angle cost
      ddp_config->set_qf(i + 15, 200.0);
      // Joint velocity cost
      ddp_config->set_qf(i + 17, 5.0);
    }
    ddp_config->set_debug(false);
    ddp_config->set_max_iters(5);
    config.set_weights_folder(
        "neural_network_model_data/tensorflow_model_vars_16_8_tanh/");
    return config;
  }

  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    /*data_config.set_stream_id("velocity_based_position_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("rpyt_based_velocity_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
    */
  }

  void runUntilConvergence(const PositionYaw &initial_state,
                           const PositionYaw &goal,
                           std::vector<double> &&goal_joint_angles,
                           std::vector<double> &&init_joint_angles = {0, 0},
                           bool check_thrust_gain = true) {
    // Fly quadrotor which sets the altitude to 0.5
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_state.x;
    init_position.y = initial_state.y;
    init_position.z = initial_state.z;
    drone_hardware_.cmdwaypoint(init_position, initial_state.yaw);
    arm_simulator_.setJointAngles(init_joint_angles);
    controller_connector_->setGoal(
        createWayPoint(goal, goal_joint_angles[0], goal_joint_angles[1]));
    auto runController = [&]() {
      controller_connector_->run();
      return controller_connector_->getStatus() == ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(
        runController, std::chrono::seconds(5), std::chrono::milliseconds(0)));
    // Check position is close to goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    tf::Transform goal_transform(tf::createQuaternionFromRPY(0, 0, goal.yaw),
                                 tf::Vector3(goal.x, goal.y, goal.z));
    test_utils::ASSERT_VEC_NEAR(quad_transform.getOrigin(),
                                goal_transform.getOrigin(),
                                controller_config_.goal_position_tolerance());
    tf::Vector3 zero_vec(0, 0, 0);
    test_utils::ASSERT_VEC_NEAR(
        tf::Vector3(sensor_data.linvel.x, sensor_data.linvel.y,
                    sensor_data.linvel.z),
        zero_vec, controller_config_.goal_velocity_tolerance());
    ControllerStatus status = controller_connector_->getStatus();
    ASSERT_EQ(status, ControllerStatus::Completed) << status.statusAsText();
    if (check_thrust_gain) {
      ASSERT_NEAR(thrust_gain_estimator_.getThrustGain(), 0.16, 1e-2);
    }
  }

  std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
  createWayPoint(PositionYaw goal, double desired_joint_angle_1,
                 double desired_joint_angle_2) {
    std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>> waypoint;
    Eigen::VectorXd goal_control(6);
    goal_control << 1, 0, 0, 0, desired_joint_angle_1, desired_joint_angle_2;
    Eigen::VectorXd goal_state(21);
    goal_state << goal.x, goal.y, goal.z, 0, 0, goal.yaw, 0, 0, 0, 0, 0, 0, 0,
        0, goal.yaw, desired_joint_angle_1, desired_joint_angle_2, 0, 0,
        desired_joint_angle_1, desired_joint_angle_2;
    waypoint.reset(new Waypoint<Eigen::VectorXd, Eigen::VectorXd>(
        goal_state, goal_control));
    return waypoint;
  }

  quad_simulator::QuadSimulator drone_hardware_;
  ArmSimulator arm_simulator_;
  std::unique_ptr<DDPAirmMPCController> controller_;
  std::unique_ptr<MPCControllerAirmConnector> controller_connector_;
  ThrustGainEstimator thrust_gain_estimator_;
  AirmMPCControllerConfig controller_config_;
};
