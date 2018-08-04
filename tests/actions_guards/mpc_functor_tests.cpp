#include <aerial_autonomy/actions_guards/mpc_states_actions.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <arm_parsers/arm_simulator.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <typeindex>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;
/**
* @brief Sample logic state machine templated on UAV arm system
*/
using UAVArmLogicStateMachine = SampleLogicStateMachine_<UAVArmSystem>;
/**
 * @brief namespace for mpc states actions
 */
using msa = MPCStatesActions<UAVArmLogicStateMachine>;

// Visual Servoing
using MPCInternalAction =
    MPCControlInternalActionFunctor_<UAVArmLogicStateMachine>;

class MPCFunctorTests : public ::testing::Test {
protected:
  std::shared_ptr<QuadSimulator> drone_hardware;
  UAVSystemConfig config;
  std::shared_ptr<SimpleTracker> simple_tracker;
  std::shared_ptr<ArmSimulator> arm_simulator;
  std::unique_ptr<UAVArmSystem> uav_system;
  std::unique_ptr<UAVArmLogicStateMachine> sample_logic_state_machine;
  BaseStateMachineConfig state_machine_config;

  MPCFunctorTests() {
    drone_hardware.reset(new QuadSimulator);
    drone_hardware->usePerfectTime();
    arm_simulator.reset(new ArmSimulator);
    ///////////////Fill UAV System Config/////////////////
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    auto position_tolerance = config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.5);
    position_tolerance->set_y(0.5);
    position_tolerance->set_z(0.5);
    auto vel_based_position_controller =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_velocity_based_position_controller_config();
    vel_based_position_controller->set_position_gain(1.0);
    vel_based_position_controller->set_max_velocity(1.0);
    vel_based_position_controller->set_yaw_gain(1);
    auto vel_position_controller_tol =
        vel_based_position_controller->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_position_controller_tol->set_x(0.1);
    vel_position_controller_tol->set_y(0.1);
    vel_position_controller_tol->set_z(0.1);
    auto rpyt_vel_controller_tol =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    rpyt_vel_controller_tol->set_vx(0.1);
    rpyt_vel_controller_tol->set_vy(0.1);
    rpyt_vel_controller_tol->set_vz(0.1);
    ///////////Create Dummy Tracker///////////
    tf::Transform camera_transform = conversions::protoTransformToTf(
        uav_vision_system_config->camera_transform());
    simple_tracker.reset(new SimpleTracker(*drone_hardware, camera_transform));
    // Fill MPC Config
    test_utils::fillMPCConfig(config);
    // Set visualization to false
    uav_vision_system_config->mutable_uav_arm_system_config()
        ->set_visualize_mpc_trajectories(false);
    uav_system.reset(new UAVArmSystem(
        config, std::dynamic_pointer_cast<BaseTracker>(simple_tracker),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware),
        arm_simulator));
    uav_system->usePerfectMPCTime();
    ////////// Fill State machine config////////////
    auto mpc_state_machine_config =
        state_machine_config.mutable_mpc_state_machine_config();
    auto arm_reference = mpc_state_machine_config->mutable_arm_reference();
    auto joint1_config = arm_reference->add_joint_config();
    auto joint2_config = arm_reference->add_joint_config();
    joint1_config->set_offset(-0.8);
    joint2_config->set_offset(1.2);
    joint1_config->set_amplitude(0.2);
    joint2_config->set_amplitude(0.2);
    joint1_config->set_frequency(0.1);
    joint2_config->set_frequency(0.1);
    joint1_config->set_phase(0);
    joint2_config->set_phase(M_PI / 2.0);
    sample_logic_state_machine.reset(
        new UAVArmLogicStateMachine(*uav_system, state_machine_config));
  }

  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("rpyt_based_velocity_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("velocity_based_position_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
  }

  virtual ~MPCFunctorTests(){};
};
/// \brief Test Visual Servoing
TEST_F(MPCFunctorTests, Constructor) {
  ASSERT_NO_THROW(new msa::MPCSpiralTransition());
  ASSERT_NO_THROW(new msa::MPCWaypointTransition());
  ASSERT_NO_THROW(new msa::MPCState());
}

// Call Internal Action function after setting the quad to the right target
// location
TEST_F(MPCFunctorTests, CallInternalActionFunction) {
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  uav_system->power(true);
  double desired_yaw = 0.5;
  geometry_msgs::Vector3 desired_position;
  desired_position.x = 0;
  desired_position.y = 0.0;
  desired_position.z = 0.5;
  drone_hardware->cmdwaypoint(desired_position, desired_yaw);
  msa::MPCSpiralTransition mpc_spiral_transition_action;
  int dummy_start_state, dummy_target_state;
  mpc_spiral_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  // Run the controller
  uav_system->runActiveController(ControllerGroup::UAV);
  // After transition the status should be active
  ControllerStatus status;
  status = uav_system->getStatus<MPCControllerAirmConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Get Goal
  auto goal =
      uav_system
          ->getGoal<MPCControllerAirmConnector,
                    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> desired_state_control_pair =
      goal->atTime(0);
  test_utils::ASSERT_VEC_NEAR(
      Eigen::Vector3d(desired_state_control_pair.first.segment<3>(0)),
      Eigen::Vector3d(0, 0, 0.5));
  ASSERT_NEAR(desired_state_control_pair.first[5], 0.5, 1e-5);
}

TEST_F(MPCFunctorTests, LowBatteryCallInternalActionFunction) {
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  uav_system->power(true);
  // Call action functor
  msa::MPCSpiralTransition mpc_spiral_transition_action;
  int dummy_start_state, dummy_target_state;
  mpc_spiral_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  // Run the controller
  uav_system->runActiveController(ControllerGroup::UAV);
  // Check status of controller
  ControllerStatus status;
  status = uav_system->getStatus<MPCControllerAirmConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Set battery voltage to low value
  drone_hardware->setBatteryPercent(10);
  // Test internal action
  MPCInternalAction mpc_internal_action;
  mpc_internal_action(NULL, *sample_logic_state_machine, dummy_start_state,
                      dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

TEST_F(MPCFunctorTests, FailedMPCInternalActionFunction) {
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  uav_system->power(true);
  // Call action functor
  msa::MPCSpiralTransition mpc_spiral_transition_action;
  int dummy_start_state, dummy_target_state;
  mpc_spiral_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  // Check status of controller
  ControllerStatus status;
  status = uav_system->getStatus<MPCControllerAirmConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Move quad far enough that optimization fails
  double desired_yaw = 0.5;
  geometry_msgs::Vector3 desired_position;
  desired_position.x = 100;
  desired_position.y = 100.0;
  desired_position.z = 0.5;
  drone_hardware->cmdwaypoint(desired_position, desired_yaw);
  // Run the controller
  uav_system->runActiveController(ControllerGroup::UAV);
  // Test internal action
  MPCInternalAction mpc_internal_action;
  mpc_internal_action(NULL, *sample_logic_state_machine, dummy_start_state,
                      dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
  status = uav_system->getStatus<MPCControllerAirmConnector>();
  ASSERT_EQ(status, ControllerStatus::Critical);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
