#include "position_yaw.pb.h"
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
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
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using psa = PickPlaceStatesActions<UAVArmLogicStateMachine>;

// Picking internal actions
using PrePickInternalAction =
    PrePickInternalActionFunctor_<UAVArmLogicStateMachine>;

using ArmFoldInternalAction =
    ArmFoldInternalActionFunctor_<UAVArmLogicStateMachine>;

using ManualControlArmAction =
    ManualControlArmInternalActionFunctor_<UAVArmLogicStateMachine>;

class PickPlaceFunctorTests : public ::testing::Test {
protected:
  QuadSimulator drone_hardware;
  UAVSystemConfig config;
  ArmSimulator arm;
  std::unique_ptr<SimpleTracker> simple_tracker;
  std::unique_ptr<UAVArmSystem> uav_arm_system;
  std::unique_ptr<UAVArmLogicStateMachine> sample_logic_state_machine;
  PickPlaceFunctorTests() {
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    auto uav_arm_system_config =
        uav_vision_system_config->mutable_uav_arm_system_config();
    for (int i = 0; i < 6; ++i) {
      uav_vision_system_config->add_camera_transform(0.0);
    }
    for (int i = 0; i < 6; ++i) {
      uav_vision_system_config->add_tracking_offset_transform(0.0);
    }
    // Flipped arm
    // Arm transform xyz(0.2,0,-0.1), rpy(pi,0,0):
    uav_arm_system_config->add_arm_transform(0.2);
    uav_arm_system_config->add_arm_transform(0);
    uav_arm_system_config->add_arm_transform(-0.1);
    uav_arm_system_config->add_arm_transform(M_PI);
    uav_arm_system_config->add_arm_transform(0);
    uav_arm_system_config->add_arm_transform(0);
    // Arm goal transform xyz(-0.2,0,0), rpy(0,0,0):
    // Pre-pick goal
    uav_arm_system_config->add_arm_goal_transform(-0.1);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    // Pick goal
    uav_arm_system_config->add_arm_goal_transform(-0.2);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    uav_arm_system_config->add_arm_goal_transform(0);
    // waypoints
    setWaypoint(uav_arm_system_config->add_way_points(), 0.1, 0, 0, 0);
    setWaypoint(uav_arm_system_config->add_way_points(), 0, 0, 0, M_PI / 2.0);
    setWaypoint(uav_arm_system_config->add_way_points(), 0, -1.0, 0,
                M_PI / 2.0);

    uav_vision_system_config->add_relative_pose_goals();
    auto pose_goal = uav_vision_system_config->add_relative_pose_goals();
    pose_goal->mutable_position()->set_x(1);
    pose_goal->mutable_position()->set_y(-1);
    pose_goal->mutable_position()->set_z(1);
    pose_goal->set_yaw(0);

    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    tf::Transform camera_transform = math::getTransformFromVector(
        uav_vision_system_config->camera_transform());
    auto depth_config =
        uav_vision_system_config
            ->mutable_constant_heading_depth_controller_config();
    auto position_tolerance = depth_config->mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(.1);
    position_tolerance->set_y(.1);
    position_tolerance->set_z(.1);
    auto arm_position_tolerance =
        uav_arm_system_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    arm_position_tolerance->set_x(.1);
    arm_position_tolerance->set_y(.1);
    arm_position_tolerance->set_z(.1);
    simple_tracker.reset(new SimpleTracker(drone_hardware, camera_transform));
    uav_arm_system.reset(
        new UAVArmSystem(*simple_tracker, drone_hardware, arm, config));
    sample_logic_state_machine.reset(
        new UAVArmLogicStateMachine(*uav_arm_system));
  }

  void setWaypoint(config::PositionYaw *way_point, double x, double y, double z,
                   double yaw) {
    way_point->mutable_position()->set_x(x);
    way_point->mutable_position()->set_y(y);
    way_point->mutable_position()->set_z(z);
    way_point->set_yaw(yaw);
  }

  virtual ~PickPlaceFunctorTests(){};
};
/// \brief Test Visual Servoing
TEST_F(PickPlaceFunctorTests, Constructor) {
  ASSERT_NO_THROW(new psa::PickTransitionAction());
  ASSERT_NO_THROW(new PrePickInternalAction());
  ASSERT_NO_THROW(new ArmFoldInternalAction());
}

TEST_F(PickPlaceFunctorTests, CallPrePickActionFunction) {
  // Specify a global position
  Position roi_goal(1, 1, 1);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Test action functor
  psa::PrePickTransitionGuard pick_place_transition_guard;
  psa::PrePickTransitionAction pick_place_transition_action;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(uav_arm_system->isHomeLocationSpecified());
  uav_arm_system->power(true);
  ASSERT_TRUE(pick_place_transition_guard(NULL, *sample_logic_state_machine,
                                          dummy_start_state,
                                          dummy_target_state));
  pick_place_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  // Arm goal
  tf::Transform arm_goal =
      uav_arm_system
          ->getGoal<VisualServoingControllerArmConnector, tf::Transform>();
  // Check origin
  ASSERT_EQ(arm_goal.getOrigin(), tf::Vector3(-0.1, 0, 0));
}

TEST_F(PickPlaceFunctorTests, CallPickActionFunction) {
  // Specify a global position
  Position roi_goal(1, 1, 1);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Test action functor
  psa::PickGuard pick_place_transition_guard;
  psa::PickTransitionAction pick_place_transition_action;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(uav_arm_system->isHomeLocationSpecified());
  uav_arm_system->power(true);
  ASSERT_TRUE(pick_place_transition_guard(NULL, *sample_logic_state_machine,
                                          dummy_start_state,
                                          dummy_target_state));
  pick_place_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  // UAV goal
  PositionYaw uav_goal = uav_arm_system->getGoal<
      RelativePoseVisualServoingControllerDroneConnector, PositionYaw>();
  // Check origin
  ASSERT_EQ(uav_goal, PositionYaw(1, -1, 1, 0));
}

TEST_F(PickPlaceFunctorTests, PoweroffCallGuardFunction) {
  // Turn off arm
  uav_arm_system->power(false);
  psa::PrePickTransitionGuard pick_place_transition_guard;
  int dummy_start_state, dummy_target_state;
  bool result = pick_place_transition_guard(
      NULL, *sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_FALSE(result);
}

// Call PrePick Internal Action function after setting the quad to the right
// target
// location and Arm to right location
TEST_F(PickPlaceFunctorTests, CallPrePickInternalActionFunction) {
  // Specify a global position
  Position roi_goal(5, 0, 0.5);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
  // Power on arm
  uav_arm_system->power(true);
  // Call action functor
  psa::PrePickTransitionAction pick_place_transition_action;
  psa::PrePickTransitionGuard pick_place_transition_guard;
  int dummy_start_state, dummy_target_state;
  ASSERT_TRUE(pick_place_transition_guard(NULL, *sample_logic_state_machine,
                                          dummy_start_state,
                                          dummy_target_state));
  pick_place_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  // After transition the status should be active
  ControllerStatus status;
  status = uav_arm_system->getStatus<VisualServoingControllerArmConnector>();
  ASSERT_EQ(status, ControllerStatus::Active);
  // Set quadrotor to the target location
  double desired_yaw = 0.0;
  geometry_msgs::Vector3 desired_position;
  desired_position.x = roi_goal.x - 1.0;
  desired_position.y = roi_goal.y;
  desired_position.z = roi_goal.z;
  drone_hardware.cmdwaypoint(desired_position, desired_yaw);
  // Set Arm to target location by running arm controller
  uav_arm_system->runActiveController(HardwareType::Arm);
  // Rerun loop to update status
  uav_arm_system->runActiveController(HardwareType::Arm);
  // Check status
  status = uav_arm_system->getStatus<VisualServoingControllerArmConnector>();
  ASSERT_EQ(status, ControllerStatus::Completed);
  // Call internal action functor
  PrePickInternalAction pick_internal_action;
  pick_internal_action(NULL, *sample_logic_state_machine, dummy_start_state,
                       dummy_target_state);
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}

TEST_F(PickPlaceFunctorTests, WaypointSequenceTransitionGuardCheck) {
  WaypointSequenceTransitionGuardFunctor_<UAVArmLogicStateMachine, 3, 5>
      guard_false;
  int dummy_start_state, dummy_target_state;
  EXPECT_FALSE(guard_false(NULL, *sample_logic_state_machine, dummy_start_state,
                           dummy_target_state));
  WaypointSequenceTransitionGuardFunctor_<UAVArmLogicStateMachine, 0, 2>
      guard_true;
  EXPECT_TRUE(guard_true(NULL, *sample_logic_state_machine, dummy_start_state,
                         dummy_target_state));
}

TEST_F(PickPlaceFunctorTests, ArmFoldInternalPoweroff) {
  drone_hardware.setBatteryPercent(60);
  uav_arm_system->power(false);
  ArmFoldInternalAction arm_fold_internal_action;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(arm_fold_internal_action(NULL, *sample_logic_state_machine,
                                        dummy_start_state, dummy_target_state));
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

TEST_F(PickPlaceFunctorTests, ArmFoldInternalCompleted) {
  drone_hardware.setBatteryPercent(60);
  uav_arm_system->power(true);
  uav_arm_system->foldArm();
  ArmFoldInternalAction arm_fold_internal_action;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(arm_fold_internal_action(NULL, *sample_logic_state_machine,
                                        dummy_start_state, dummy_target_state));
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(Completed)));
}

TEST_F(PickPlaceFunctorTests, ManualAction) {
  // Fly quadrotor which sets the altitude to 0.5
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
  drone_hardware.flowControl(false);
  // Check arm is powered off
  ASSERT_FALSE(uav_arm_system->enabled());
  // Call action functor
  ManualControlArmAction manual_control_arm_action;
  int dummy_start_state, dummy_target_state;
  manual_control_arm_action(NULL, *sample_logic_state_machine,
                            dummy_start_state, dummy_target_state);
  // Since arm is not enabled it will create an abort event
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
  // Turn on arm
  uav_arm_system->power(true);
  // Turn drone flow control on
  drone_hardware.flowControl(true);
  // Rerun action functor
  manual_control_arm_action(NULL, *sample_logic_state_machine,
                            dummy_start_state, dummy_target_state);
  // Check event type
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Takeoff)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
