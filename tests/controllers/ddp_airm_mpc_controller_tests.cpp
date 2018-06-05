#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/spiral_reference_trajectory.h"
#include "aerial_autonomy/types/waypoint.h"
#include "ddp_mpc_controller_config.pb.h"

#include <gtest/gtest.h>

class DDPAirmMPCControllerTests : public ::testing::Test {
public:
  using AirmGcopWaypoint = Waypoint<Eigen::VectorXd, Eigen::VectorXd>;

  DDPAirmMPCControllerTests() {
    DDPMPCControllerConfig *ddp_config = config_.mutable_ddp_config();
    ddp_config->set_look_ahead_time(0.02);
    auto q = ddp_config->mutable_q();
    q->Resize(21, 0.0);
    auto qf = ddp_config->mutable_qf();
    qf->Resize(21, 0.1);
    auto r = ddp_config->mutable_r();
    r->Resize(6, 0.1);
    r->Set(0, 10.0); // Reduce thrust control
    for (int i = 0; i < 3; ++i) {
      // Pos cost
      ddp_config->set_qf(i, 100.0);
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
      ddp_config->set_qf(i + 15, 20.0);
      // Joint velocity cost
      ddp_config->set_qf(i + 17, 1.0);
    }
    ddp_config->set_debug(false);
    ddp_config->set_max_iters(100);
    config_.set_weights_folder(
        std::string(PROJECT_SOURCE_DIR) +
        "/neural_network_model_data/tensorflow_model_vars_16_8_tanh/");
  }

  std::shared_ptr<DDPAirmMPCController>
  createController(double controller_duration = 0.02) {
    std::shared_ptr<DDPAirmMPCController> controller(new DDPAirmMPCController(
        config_, std::chrono::duration<double>(controller_duration)));
    return controller;
  }

  void checkState(Eigen::VectorXd &state, Eigen::VectorXd &goal_state) {
    ASSERT_EQ(state.rows(), 21);
    ASSERT_EQ(goal_state.rows(), 21);
    // Check positions are close enough
    for (int i = 0; i < 3; ++i) {
      ASSERT_NEAR(state[i], goal_state[i], 1e-1);         // pos
      ASSERT_NEAR(state[i + 3], goal_state[i + 3], 5e-2); // rpy
      ASSERT_NEAR(state[i + 6], goal_state[i + 6], 0.2);  // vel
      ASSERT_NEAR(state[i + 9], goal_state[i + 9], 0.1);  // rpydot
    }
  }

  void checkState(Eigen::VectorXd &state) {
    ASSERT_EQ(state.rows(), 21);
    for (int i = 0; i < 3; ++i) {
      ASSERT_FALSE(std::isnan(state[i]));      // position
      ASSERT_LT(std::abs(state[i + 6]), 3.0);  // velocity
      ASSERT_LT(std::abs(state[i + 9]), M_PI); // rpydot
    }
    for (int i = 0; i < 2; ++i) {
      ASSERT_LT(std::abs(state[i + 3]), M_PI / 4);  // rp
      ASSERT_LT(std::abs(state[i + 12]), M_PI / 4); // rp_cmd
      ASSERT_FALSE(std::isnan(state[i + 15]));      // joint angles
      ASSERT_LT(std::abs(state[17 + i]), M_PI);     // joint velocities
      ASSERT_FALSE(std::isnan(state[i + 19]));      // joint angles cmd
    }
    ASSERT_FALSE(std::isnan(state[5])); // yaw
  }

  void checkControl(Eigen::VectorXd &control) {
    ASSERT_EQ(control.rows(), 6);
    ASSERT_LT(control[0], 2.0);
    ASSERT_GT(control[0], 0.1);
    for (int i = 0; i < 3; ++i) {
      ASSERT_LT(std::abs(control[i + 1]), 2 * M_PI); // rpy_dot
    }
  }

  void checkOutControl(Eigen::VectorXd &control) {
    ASSERT_EQ(control.rows(), 6);
    ASSERT_LT(control[0], 2.0 * 9.81 / 0.16);
    ASSERT_GT(control[0], 0.1 * 9.81 / 0.16);
    for (int i = 0; i < 2; ++i) {
      ASSERT_LT(std::abs(control[i + 1]), M_PI / 4); // rp
    }
    ASSERT_LT(std::abs(control[4]), M_PI); // yaw rate
  }

protected:
  AirmMPCControllerConfig config_;
};

TEST_F(DDPAirmMPCControllerTests, ConstructorWithoutResidualDynamics) {
  ASSERT_NO_THROW(createController());
}

TEST_F(DDPAirmMPCControllerTests, ConstructorWithResidualDynamics) {
  config_.set_use_residual_dynamics(false);
  ASSERT_NO_THROW(createController());
}

TEST_F(DDPAirmMPCControllerTests, SingleRunWithResidualDynamics) {
  config_.set_use_residual_dynamics(true);
  config_.mutable_ddp_config()->set_n(100);
  config_.mutable_ddp_config()->set_min_cost(20);
  config_.mutable_ddp_config()->set_min_cost_decrease(1e-3);
  std::shared_ptr<DDPAirmMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(21, 1, 0);
  sensor_data.initial_state[15] = -0.8;
  sensor_data.initial_state[16] = 1.4;
  sensor_data.initial_state[19] = sensor_data.initial_state[15];
  sensor_data.initial_state[20] = sensor_data.initial_state[16];
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.1; // Some random value does not matter
  // Set Goal
  Eigen::VectorXd goal_state(21);
  VLOG(1) << "Creating goal";
  goal_state.setZero();
  goal_state[0] = goal_state[1] = goal_state[2] = 1.0;
  goal_state[5] = goal_state[14] = 0.5; // Yaw, yawd
  // Joint angles
  goal_state[15] = -0.3;
  goal_state[16] = 0.7;
  goal_state[19] = goal_state[15]; // Desired joint angles
  goal_state[20] = goal_state[16];
  Eigen::VectorXd goal_control(6);
  goal_control.setZero();
  goal_control[0] = 1.0;
  std::shared_ptr<AirmGcopWaypoint> way_point(
      new AirmGcopWaypoint(goal_state, goal_control));
  controller->setGoal(way_point);
  Eigen::VectorXd out_control;
  VLOG(1) << "Running Controller";
  controller->run(sensor_data, out_control);
  VLOG(1) << "Done optimizing";
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  controller->getTrajectory(xs_out, us_out);
  int N = us_out.size();
  // Check states and controls are meaningful
  for (int i = 0; i < N; ++i) {
    checkState(xs_out[i]);
    checkControl(us_out[i]);
  }
  // Terminal state
  checkState(xs_out[N], goal_state);
  checkOutControl(out_control);
  // Check control based on command
  ASSERT_GT(out_control[0], 1.0 * 9.81 / 0.16);
  ASSERT_LT(out_control[1], 0);
  ASSERT_GT(out_control[2], 0);
  ASSERT_GT(out_control[3], 0);
  // Joint des angles
  ASSERT_GT(out_control[4], sensor_data.initial_state[15]);
  ASSERT_LT(out_control[5], sensor_data.initial_state[16]);
}

TEST_F(DDPAirmMPCControllerTests, Convergence) {
  config_.set_use_residual_dynamics(true);
  config_.mutable_ddp_config()->set_n(50);
  config_.mutable_ddp_config()->set_min_cost(20);
  config_.mutable_ddp_config()->set_min_cost_decrease(1e-2);
  config_.mutable_ddp_config()->set_max_iters(5);
  std::shared_ptr<DDPAirmMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(21, 1, 0);
  sensor_data.initial_state[15] = -0.8;
  sensor_data.initial_state[16] = 1.4;
  sensor_data.initial_state[19] = sensor_data.initial_state[15];
  sensor_data.initial_state[20] = sensor_data.initial_state[16];
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.0; // Some random value does not matter
  // Set Goal
  Eigen::VectorXd goal_state(21);
  VLOG(1) << "Creating goal";
  goal_state.setZero();
  goal_state[0] = goal_state[1] = goal_state[2] = 1.0;
  goal_state[5] = goal_state[14] = 0.5; // Yaw, yawd
  // Joint angles
  goal_state[15] = -0.3;
  goal_state[16] = 0.7;
  goal_state[19] = goal_state[15]; // Desired joint angles
  goal_state[20] = goal_state[16];
  Eigen::VectorXd goal_control(6);
  goal_control.setZero();
  goal_control[0] = 1.0;
  std::shared_ptr<AirmGcopWaypoint> way_point(
      new AirmGcopWaypoint(goal_state, goal_control));
  controller->setGoal(way_point);
  Eigen::VectorXd out_control;
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  // Try following the trajectory perfectly
  while (sensor_data.time_since_goal < 10 &&
         !controller->isConverged(sensor_data)) {
    controller->run(sensor_data, out_control);
    controller->getTrajectory(xs_out, us_out);
    sensor_data.time_since_goal += 0.02;
    sensor_data.initial_state = xs_out[1];
    VLOG(1) << "Loop period: " << controller->getLoopTime();
  }
  ASSERT_EQ(controller->isConverged(sensor_data), ControllerStatus::Completed);
  controller->getTrajectory(xs_out, us_out);
  checkState(xs_out[0], goal_state);
}

TEST_F(DDPAirmMPCControllerTests, ConvergenceSpiral) {
  config_.set_use_residual_dynamics(true);
  config_.mutable_ddp_config()->set_n(50);
  config_.mutable_ddp_config()->set_min_cost(30);
  config_.mutable_ddp_config()->set_min_cost_decrease(1e-2);
  config_.mutable_ddp_config()->set_max_iters(5);
  std::shared_ptr<DDPAirmMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(21, 1, 0);
  sensor_data.initial_state[15] = -0.8;
  sensor_data.initial_state[16] = 1.4;
  sensor_data.initial_state[19] = sensor_data.initial_state[15];
  sensor_data.initial_state[20] = sensor_data.initial_state[16];
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.0; // Some random value does not matter
  // Set Goal
  SpiralReferenceTrajectoryConfig reference_config;
  ArmSineControllerConfig arm_reference_config;
  auto joint1_config = arm_reference_config.add_joint_config();
  auto joint2_config = arm_reference_config.add_joint_config();
  joint1_config->set_amplitude(1.0);
  joint2_config->set_amplitude(0.5);

  joint1_config->set_frequency(0.1);
  joint2_config->set_frequency(0.2);

  joint1_config->set_offset(1.0);
  joint2_config->set_offset(2.0);

  joint1_config->set_phase(0.0);
  joint2_config->set_phase(M_PI / 2.0);

  std::shared_ptr<SpiralReferenceTrajectory> reference_trajectory(
      new SpiralReferenceTrajectory(reference_config, arm_reference_config,
                                    sensor_data.initial_state.segment<3>(0), 0,
                                    sensor_data.parameters[0]));
  controller->setGoal(reference_trajectory);
  Eigen::VectorXd out_control;
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  // Initialize by running DDP
  controller->run(sensor_data, out_control);
  // Try following the trajectory perfectly
  while (sensor_data.time_since_goal < 10 &&
         !controller->isConverged(sensor_data)) {
    controller->run(sensor_data, out_control);
    controller->getTrajectory(xs_out, us_out);
    sensor_data.time_since_goal += 0.02;
    sensor_data.initial_state = xs_out[1];
    VLOG(1) << "Loop period: " << controller->getLoopTime();
  }
  ASSERT_EQ(controller->isConverged(sensor_data), ControllerStatus::Completed);
  controller->getTrajectory(xs_out, us_out);
  std::pair<Eigen::VectorXd, Eigen::VectorXd> goal_state_control_pair =
      reference_trajectory->atTime(sensor_data.time_since_goal);
  checkState(xs_out[0], goal_state_control_pair.first);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
