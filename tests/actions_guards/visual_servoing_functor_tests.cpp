#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <typeindex>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;
using vsa = VisualServoingStatesActions<UAVVisionLogicStateMachine>;

// Visual Servoing
using VisualServoingInternalAction =
    VisualServoingInternalActionFunctor_<UAVVisionLogicStateMachine>;

using RelativePoseVisualServoingInternalAction =
    RelativePoseVisualServoingInternalActionFunctor_<UAVVisionLogicStateMachine,
                                                     be::Abort>;

class VisualServoingTests : public ::testing::Test {
protected:
  std::shared_ptr<QuadSimulator> drone_hardware;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;
  std::shared_ptr<SimpleTracker> simple_tracker;
  std::unique_ptr<UAVVisionSystem> uav_system;
  std::unique_ptr<UAVVisionLogicStateMachine> sample_logic_state_machine;
  VisualServoingTests() {
    drone_hardware.reset(new QuadSimulator);
    drone_hardware->usePerfectTime();
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    tf::Transform camera_transform = conversions::protoTransformToTf(
        uav_vision_system_config->camera_transform());
    auto position_tolerance = config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.5);
    position_tolerance->set_y(0.5);
    position_tolerance->set_z(0.5);
    auto vs_position_tolerance =
        uav_vision_system_config
            ->mutable_constant_heading_depth_controller_config()
            ->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vs_position_tolerance->set_x(0.5);
    vs_position_tolerance->set_y(0.5);
    vs_position_tolerance->set_z(0.5);

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

    auto relative_pose_vs_position_tolerance =
        uav_vision_system_config
            ->mutable_rpyt_based_relative_pose_controller_config()
            ->mutable_velocity_based_relative_pose_controller_config()
            ->mutable_velocity_based_position_controller_config()
            ->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    relative_pose_vs_position_tolerance->set_x(0.1);
    relative_pose_vs_position_tolerance->set_y(0.1);
    relative_pose_vs_position_tolerance->set_z(0.1);
    auto relative_pose_vs_velocity_tolerance =
        uav_vision_system_config
            ->mutable_rpyt_based_relative_pose_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    relative_pose_vs_velocity_tolerance->set_vx(0.1);
    relative_pose_vs_velocity_tolerance->set_vy(0.1);
    relative_pose_vs_velocity_tolerance->set_vz(0.1);
    auto pose_goal =
        state_machine_config.mutable_visual_servoing_state_machine_config()
            ->add_relative_pose_goals();
    auto pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(1);
    pose_goal_position->set_y(1);
    pose_goal_position->set_z(2);
    pose_goal->set_yaw(0);

    simple_tracker.reset(new SimpleTracker(*drone_hardware, camera_transform));
    uav_system.reset(new UAVVisionSystem(
        config, std::dynamic_pointer_cast<BaseTracker>(simple_tracker),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware)));
    sample_logic_state_machine.reset(
        new UAVVisionLogicStateMachine(*uav_system, state_machine_config));
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
    data_config.set_stream_id("rpyt_relative_pose_visual_servoing_connector");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("velocity_based_relative_pose_controller");
    Log::instance().addDataStream(data_config);
  }

  virtual ~VisualServoingTests(){};
};
/// \brief Test Visual Servoing
TEST_F(VisualServoingTests, Constructor) {
  ASSERT_NO_THROW(new vsa::VisualServoingTransitionAction());
  ASSERT_NO_THROW(new VisualServoingInternalAction());
  ASSERT_NO_THROW(new vsa::RelativePoseVisualServoingTransitionAction());
  ASSERT_NO_THROW(new RelativePoseVisualServoingInternalAction());
}

TEST_F(VisualServoingTests, CallGuardFunction) {
  // Specify a global position
  Position roi_goal(1, 1, 1);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Test action functors
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(uav_system->isHomeLocationSpecified());
  visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                   dummy_start_state, dummy_target_state);
  ASSERT_TRUE(uav_system->isHomeLocationSpecified());
  ASSERT_EQ(uav_system->getHomeLocation(), PositionYaw(0, 0, 0, 0));
  // Get goal from controllerConnector and check if the goal is what we expect
  Position goal =
      uav_system->getGoal<VisualServoingControllerDroneConnector, Position>();
  ASSERT_EQ(goal, roi_goal / roi_goal.norm());
}

TEST_F(VisualServoingTests, CallRelativePoseGuardFunction) {
  // Specify a global position
  Position roi_goal(1, 1, 1);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Test action functors
  vsa::RelativePoseVisualServoingTransitionGuard
      visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_TRUE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
}

TEST_F(VisualServoingTests, InvalidTrackingCallGuardFunction) {
  simple_tracker->setTrackingIsValid(false);
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
}

TEST_F(VisualServoingTests, InvalidTrackingRelativePoseGuardFunction) {
  simple_tracker->setTrackingIsValid(false);
  // Test guard functors
  vsa::RelativePoseVisualServoingTransitionGuard
      visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
}

TEST_F(VisualServoingTests, ZeroLengthTrackingCallGuardFunction) {
  // The tracking vector is of length zero
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
}

// Call Internal Action function after setting the quad to the right target
// location
TEST_F(VisualServoingTests, CallInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  // Call guard
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_TRUE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
  // After transition the status should be active
  ControllerStatus status;
  status = uav_system->getStatus<VisualServoingControllerDroneConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Get Goal
  Position goal =
      uav_system->getGoal<VisualServoingControllerDroneConnector, Position>();
  ASSERT_EQ(goal, Position(1, 0, 0));
  // Set quadrotor to the target location
  double desired_yaw = 0.0;
  geometry_msgs::Vector3 desired_position;
  desired_position.x = roi_goal.x - 1.0;
  desired_position.y = roi_goal.y;
  desired_position.z = roi_goal.z;
  drone_hardware->cmdwaypoint(desired_position, desired_yaw);
  // Call controller loop:
  uav_system->runActiveController(ControllerGroup::UAV);
  // Get status
  status = uav_system->getStatus<VisualServoingControllerDroneConnector>();
  ASSERT_EQ(status, ControllerStatus::Completed);
  // Call internal action functor
  VisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}

TEST_F(VisualServoingTests, CallRelativePoseInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  // Call guard
  vsa::RelativePoseVisualServoingTransitionAction
      visual_servoing_transition_action;
  int dummy_start_state, dummy_target_state;

  visual_servoing_transition_action(NULL, *sample_logic_state_machine,
                                    dummy_start_state, dummy_target_state);
  // After transition the status should be active
  ControllerStatus status;
  status = uav_system->getStatus<RPYTRelativePoseVisualServoingConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Get Goal
  Position goal =
      uav_system->getGoal<RPYTRelativePoseVisualServoingConnector, Position>();
  PositionYaw desired_relative_pose(1, 1, 2, 0.0);
  ASSERT_EQ(goal, desired_relative_pose);
  // Set quadrotor to the target location
  geometry_msgs::Vector3 desired_position;
  desired_position.x = roi_goal.x + desired_relative_pose.x;
  desired_position.y = roi_goal.y + desired_relative_pose.y;
  desired_position.z = roi_goal.z + desired_relative_pose.z;
  drone_hardware->cmdwaypoint(desired_position, desired_relative_pose.yaw);
  // Call controller loop:
  uav_system->runActiveController(ControllerGroup::UAV);
  // Get status
  status = uav_system->getStatus<RPYTRelativePoseVisualServoingConnector>();
  ASSERT_EQ(status, ControllerStatus::Completed);
  // Call internal action functor
  RelativePoseVisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}

TEST_F(VisualServoingTests, LowBatteryCallInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  // Call action functor
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                   dummy_start_state, dummy_target_state);
  // Run Internal action
  VisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  // Check status of controller
  ControllerStatus status;
  status = uav_system->getStatus<VisualServoingControllerDroneConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Set battery voltage to low value
  drone_hardware->setBatteryPercent(10);
  // Test internal action
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

TEST_F(VisualServoingTests, LowBatteryCallRelativePoseInternalActionFunction) {
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  // Call action functor
  vsa::RelativePoseVisualServoingTransitionAction
      visual_servoing_transition_action;
  int dummy_start_state, dummy_target_state;
  visual_servoing_transition_action(NULL, *sample_logic_state_machine,
                                    dummy_start_state, dummy_target_state);
  // Run Internal action
  RelativePoseVisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  // Check status of controller
  ControllerStatus status;
  status = uav_system->getStatus<RPYTRelativePoseVisualServoingConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Set battery voltage to low value
  drone_hardware->setBatteryPercent(10);
  // Test internal action
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

TEST_F(VisualServoingTests, LostTrackingInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  // Call action functor
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_TRUE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
  // Make tracking invalid
  simple_tracker->setTrackingIsValid(false);
  // Run controller to update controller status
  uav_system->runActiveController(ControllerGroup::UAV);
  // Test internal action
  VisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

TEST_F(VisualServoingTests, LostTrackingRelativePoseInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  // Call action functor
  vsa::RelativePoseVisualServoingTransitionAction
      visual_servoing_transition_action;
  int dummy_start_state, dummy_target_state;
  visual_servoing_transition_action(NULL, *sample_logic_state_machine,
                                    dummy_start_state, dummy_target_state);
  // Make tracking invalid
  simple_tracker->setTrackingIsValid(false);
  // Run controller to update controller status
  uav_system->runActiveController(ControllerGroup::UAV);
  // Test internal action
  RelativePoseVisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

// Call GoHome functions
TEST_F(VisualServoingTests, CallGoHomeTransitionAction) {
  // Takeoff
  drone_hardware->takeoff();
  // Send to a desired location
  geometry_msgs::Vector3 desired_home_position;
  desired_home_position.x = 1.0;
  desired_home_position.y = 2.0;
  desired_home_position.z = 3.0;
  drone_hardware->cmdwaypoint(desired_home_position);
  // Save home location
  uav_system->setHomeLocation();
  // Go to another location to test going back to home
  geometry_msgs::Vector3 random_location;
  random_location.x = random_location.y = random_location.z = 0;
  drone_hardware->cmdwaypoint(random_location);
  // Call gohome action
  vsa::GoHomeTransitionAction go_home_transition_action;
  int dummy_start_state, dummy_target_state;
  go_home_transition_action(NULL, *sample_logic_state_machine,
                            dummy_start_state, dummy_target_state);
  auto getUAVStatusRunController = [&]() {
    uav_system->runActiveController(ControllerGroup::UAV);
    return uav_system->getStatus<RPYTBasedPositionControllerDroneConnector>() ==
           ControllerStatus::Completed;
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(getUAVStatusRunController,
                                          std::chrono::seconds(5),
                                          std::chrono::milliseconds(0)));
  // Also check the current position
  auto uav_data = uav_system->getUAVData();
  ASSERT_NEAR(uav_data.localpos.x, desired_home_position.x, 0.1);
  ASSERT_NEAR(uav_data.localpos.y, desired_home_position.y, 0.1);
  ASSERT_NEAR(uav_data.localpos.z, desired_home_position.z, 0.1);
}

TEST_F(VisualServoingTests, GoHomeTransitionGuard) {
  // Call gohome guard
  vsa::GoHomeTransitionGuard go_home_transition_guard;
  int dummy_start_state, dummy_target_state;
  bool result = go_home_transition_guard(NULL, *sample_logic_state_machine,
                                         dummy_start_state, dummy_target_state);
  ASSERT_FALSE(result);
  // Save home location
  uav_system->setHomeLocation();
  // Check result again
  result = go_home_transition_guard(NULL, *sample_logic_state_machine,
                                    dummy_start_state, dummy_target_state);
  ASSERT_TRUE(result);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
