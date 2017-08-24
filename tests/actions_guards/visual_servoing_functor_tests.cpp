#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
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

class VisualServoingTests : public ::testing::Test {
protected:
  QuadSimulator drone_hardware;
  UAVSystemConfig config;
  std::unique_ptr<SimpleTracker> simple_tracker;
  std::unique_ptr<UAVVisionSystem> uav_system;
  std::unique_ptr<UAVVisionLogicStateMachine> sample_logic_state_machine;
  VisualServoingTests() {
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    for (int i = 0; i < 6; ++i) {
      uav_vision_system_config->add_camera_transform(0.0);
    }
    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    tf::Transform camera_transform = math::getTransformFromVector(
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
    simple_tracker.reset(new SimpleTracker(drone_hardware, camera_transform));
    uav_system.reset(
        new UAVVisionSystem(*simple_tracker, drone_hardware, config));
    sample_logic_state_machine.reset(
        new UAVVisionLogicStateMachine(*uav_system));
  }
  virtual ~VisualServoingTests(){};
};
/// \brief Test Visual Servoing
TEST_F(VisualServoingTests, Constructor) {
  ASSERT_NO_THROW(new vsa::VisualServoingTransitionAction());
  ASSERT_NO_THROW(new VisualServoingInternalAction());
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

TEST_F(VisualServoingTests, InvalidTrackingCallGuardFunction) {
  simple_tracker->setTrackingIsValid(false);
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
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
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
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
  drone_hardware.cmdwaypoint(desired_position, desired_yaw);
  // Call controller loop:
  uav_system->runActiveController(HardwareType::UAV);
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

TEST_F(VisualServoingTests, LowBatteryCallInternalActionFunction) {
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
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
  drone_hardware.setBatteryPercent(10);
  // Test internal action
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Land)));
}

TEST_F(VisualServoingTests, LostTrackingInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
  // Call action functor
  vsa::VisualServoingTransitionGuard visual_servoing_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_TRUE(
      visual_servoing_transition_guard(NULL, *sample_logic_state_machine,
                                       dummy_start_state, dummy_target_state));
  // Make tracking invalid
  simple_tracker->setTrackingIsValid(false);
  // Run controller to update controller status
  uav_system->runActiveController(HardwareType::UAV);
  // Test internal action
  VisualServoingInternalAction visual_servoing_internal_action;
  visual_servoing_internal_action(NULL, *sample_logic_state_machine,
                                  dummy_start_state, dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

// Call GoHome functions
TEST_F(VisualServoingTests, CallGoHomeTransitionAction) {
  // Takeoff
  drone_hardware.takeoff();
  // Send to a desired location
  geometry_msgs::Vector3 desired_home_position;
  desired_home_position.x = 1.0;
  desired_home_position.y = 2.0;
  desired_home_position.z = 3.0;
  drone_hardware.cmdwaypoint(desired_home_position);
  // Save home location
  uav_system->setHomeLocation();
  // Go to another location to test going back to home
  geometry_msgs::Vector3 random_location;
  random_location.x = random_location.y = random_location.z = 0;
  drone_hardware.cmdwaypoint(random_location);
  // Call gohome action
  vsa::GoHomeTransitionAction go_home_transition_action;
  int dummy_start_state, dummy_target_state;
  go_home_transition_action(NULL, *sample_logic_state_machine,
                            dummy_start_state, dummy_target_state);
  // Run the active controller once
  uav_system->runActiveController(HardwareType::UAV);
  // Run the active controller again to update controller status
  uav_system->runActiveController(HardwareType::UAV);
  // Get status to verify we are done
  ControllerStatus status =
      uav_system->getStatus<PositionControllerDroneConnector>();
  ASSERT_EQ(status, ControllerStatus::Completed);
  // Also check the current position
  auto uav_data = uav_system->getUAVData();
  ASSERT_EQ(uav_data.localpos.x, desired_home_position.x);
  ASSERT_EQ(uav_data.localpos.y, desired_home_position.y);
  ASSERT_EQ(uav_data.localpos.z, desired_home_position.z);
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
