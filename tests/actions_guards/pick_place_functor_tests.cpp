#include "position_yaw.pb.h"
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
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
* @brief Namespace fortest utilities
*/
using namespace test_utils;

/**
* @brief Sample logic state machine templated on UAV arm system
*/
using UAVArmLogicStateMachine = SampleLogicStateMachine_<UAVArmSystem>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using psa = PickPlaceStatesActions<UAVArmLogicStateMachine>;

// Picking internal actions

using ArmFoldInternalAction =
    ArmFoldInternalActionFunctor_<UAVArmLogicStateMachine>;

using ManualControlArmAction =
    ManualControlArmInternalActionFunctor_<UAVArmLogicStateMachine>;

class PickPlaceFunctorTests : public ::testing::Test {
protected:
  std::shared_ptr<QuadSimulator> drone_hardware;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;
  std::shared_ptr<SimpleTracker> simple_tracker;
  std::unique_ptr<UAVArmSystem> uav_arm_system;
  std::unique_ptr<UAVArmLogicStateMachine> sample_logic_state_machine;
  PickPlaceFunctorTests() : pose_goal_(1, -1, 1, 0) {
    drone_hardware.reset(new QuadSimulator);
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    auto uav_arm_system_config =
        uav_vision_system_config->mutable_uav_arm_system_config();
    auto pick_state_machine_config =
        state_machine_config.mutable_visual_servoing_state_machine_config()
            ->mutable_pick_place_state_machine_config();
    // Arm transform
    setTransform(uav_arm_system_config->mutable_arm_transform(), 0.2, 0, -0.1,
                 M_PI, 0, 0);
    // Arm goal transforms
    // Pick goal
    setTransform(pick_state_machine_config->add_arm_goal_transform(), -0.1, 0,
                 0, 0, 0, 0);
    // waypoints
    auto waypoint_config =
        pick_state_machine_config->mutable_following_waypoint_sequence_config();
    setWaypoint(waypoint_config->add_way_points(), 0.1, 0, 0, 0);
    setWaypoint(waypoint_config->add_way_points(), 0, 0, 0, M_PI / 2.0);
    setWaypoint(waypoint_config->add_way_points(), 0, -1.0, 0, M_PI / 2.0);

    auto vision_state_machine_config =
        state_machine_config.mutable_visual_servoing_state_machine_config();
    auto pose_goal = vision_state_machine_config->add_relative_pose_goals();
    pose_goal->mutable_position()->set_x(pose_goal_.x);
    pose_goal->mutable_position()->set_y(pose_goal_.y);
    pose_goal->mutable_position()->set_z(pose_goal_.z);
    pose_goal->set_yaw(pose_goal_.yaw);

    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    tf::Transform camera_transform = conversions::protoTransformToTf(
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
    // Fill quad mpc config
    auto mpc_config = config.mutable_quad_mpc_controller_config();
    test_utils::fillQuadMPCConfig(*mpc_config);
    // Fill MPC Config
    test_utils::fillMPCConfig(config);
    simple_tracker.reset(new SimpleTracker(*drone_hardware, camera_transform));
    uav_arm_system.reset(new UAVArmSystem(
        config, std::dynamic_pointer_cast<BaseTracker>(simple_tracker),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware)));
    sample_logic_state_machine.reset(
        new UAVArmLogicStateMachine(*uav_arm_system, state_machine_config));
  }

  void setTransform(config::Transform *tf, double x, double y, double z,
                    double roll, double pitch, double yaw) {
    tf->mutable_position()->set_x(x);
    tf->mutable_position()->set_y(y);
    tf->mutable_position()->set_z(z);
    tf->mutable_rotation()->set_r(roll);
    tf->mutable_rotation()->set_p(pitch);
    tf->mutable_rotation()->set_y(yaw);
  }
  void setWaypoint(config::PositionYaw *way_point, double x, double y, double z,
                   double yaw) {
    way_point->mutable_position()->set_x(x);
    way_point->mutable_position()->set_y(y);
    way_point->mutable_position()->set_z(z);
    way_point->set_yaw(yaw);
  }

  virtual ~PickPlaceFunctorTests(){};

protected:
  PositionYaw pose_goal_;
};
/// \brief Test Visual Servoing
TEST_F(PickPlaceFunctorTests, Constructor) {
  ASSERT_NO_THROW(new psa::PickTransitionAction());
  ASSERT_NO_THROW(new ArmFoldInternalAction());
}

TEST_F(PickPlaceFunctorTests, CallPickActionFunction) {
  // Specify a global position
  Position roi_goal(1, 1, 1);
  simple_tracker->setTargetPositionGlobalFrame(roi_goal);
  // Test action functor
  psa::PickTransitionGuard pick_place_transition_guard;
  psa::PickTransitionAction pick_place_transition_action;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(uav_arm_system->isHomeLocationSpecified());
  uav_arm_system->power(true);
  ASSERT_TRUE(pick_place_transition_guard(NULL, *sample_logic_state_machine,
                                          dummy_start_state,
                                          dummy_target_state));
  pick_place_transition_action(NULL, *sample_logic_state_machine,
                               dummy_start_state, dummy_target_state);
  ASSERT_TRUE(uav_arm_system->isHomeLocationSpecified());
  // UAV goal
  PositionYaw uav_goal = uav_arm_system->getGoal<
      UAVVisionSystem::VisualServoingReferenceConnectorT, PositionYaw>();
  // Arm goal
  tf::Transform arm_goal =
      uav_arm_system
          ->getGoal<BuiltInPoseControllerArmConnector, tf::Transform>();
  // Check goals
  ASSERT_EQ(uav_goal, pose_goal_);
  ASSERT_TF_NEAR(arm_goal, tf::Transform(tf::Quaternion(0, 0, 0, 1),
                                         tf::Vector3(-0.1, 0, 0)));
}

TEST_F(PickPlaceFunctorTests, PoweroffCallGuardFunction) {
  // Turn off arm
  uav_arm_system->power(false);
  psa::PickTransitionGuard pick_place_transition_guard;
  int dummy_start_state, dummy_target_state;
  bool result = pick_place_transition_guard(
      NULL, *sample_logic_state_machine, dummy_start_state, dummy_target_state);
  ASSERT_FALSE(result);
}

TEST_F(PickPlaceFunctorTests, ArmFoldInternalPoweroff) {
  drone_hardware->setBatteryPercent(60);
  uav_arm_system->power(false);
  ArmFoldInternalAction arm_fold_internal_action;
  int dummy_start_state, dummy_target_state;
  ASSERT_FALSE(arm_fold_internal_action(NULL, *sample_logic_state_machine,
                                        dummy_start_state, dummy_target_state));
  ASSERT_EQ(sample_logic_state_machine->getProcessEventTypeId(),
            std::type_index(typeid(be::Abort)));
}

TEST_F(PickPlaceFunctorTests, ArmFoldInternalCompleted) {
  drone_hardware->setBatteryPercent(60);
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
  drone_hardware->setBatteryPercent(60);
  drone_hardware->takeoff();
  drone_hardware->flowControl(false);
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
  drone_hardware->flowControl(true);
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
