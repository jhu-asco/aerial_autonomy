#include <aerial_autonomy/state_machines/pick_place_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <arm_parsers/arm_simulator.h>
#include <gtest/gtest.h>
// Timer stuff
#include <chrono>
#include <thread>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief Visual servoing events such as TrackROI, GoHome
*/
namespace vse = visual_servoing_events;

/**
* @brief Namespace for pick place events
*/
namespace pe = pick_place_events;

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

class PickPlaceStateMachineTests : public ::testing::Test {
public:
  PickPlaceStateMachineTests() : goal_tolerance_position(0.5) {
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    for (int i = 0; i < 6; ++i) {
      uav_vision_system_config->add_camera_transform(0.0);
      uav_vision_system_config->add_tracking_offset_transform(0.0);
    }
    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    auto uav_arm_system_config =
        uav_vision_system_config->mutable_uav_arm_system_config();
    for (int i = 0; i < 6; ++i) {
      uav_arm_system_config->add_arm_transform(0);
    }
    for (int i = 0; i < 12; ++i) {
      uav_arm_system_config->add_arm_goal_transform(0);
    }
    auto depth_config =
        uav_vision_system_config
            ->mutable_constant_heading_depth_controller_config();
    depth_config->set_radial_gain(0.5);
    auto position_tolerance = depth_config->mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(goal_tolerance_position);
    position_tolerance->set_y(goal_tolerance_position);
    position_tolerance->set_z(goal_tolerance_position);

    auto vel_based_pos_controller_config =
        uav_vision_system_config
            ->mutable_velocity_based_relative_pose_controller_config()
            ->mutable_velocity_based_position_controller_config();
    vel_based_pos_controller_config->set_position_gain(10.);
    auto relative_pose_vs_position_tolerance =
        vel_based_pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_based_pos_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(0.1);
    relative_pose_vs_position_tolerance->set_x(0.1);
    relative_pose_vs_position_tolerance->set_y(0.1);
    relative_pose_vs_position_tolerance->set_z(0.1);
    auto pose_goal = uav_vision_system_config->add_relative_pose_goals();
    auto pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(1);
    pose_goal_position->set_y(1);
    pose_goal_position->set_z(2);
    pose_goal->set_yaw(0);
    uav_vision_system_config->add_relative_pose_goals();

    auto arm_position_tolerance =
        uav_arm_system_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    arm_position_tolerance->set_x(goal_tolerance_position);
    arm_position_tolerance->set_y(goal_tolerance_position);
    arm_position_tolerance->set_z(goal_tolerance_position);
    tf::Transform camera_transform = math::getTransformFromVector(
        uav_vision_system_config->camera_transform());
    tracker.reset(new SimpleTracker(drone_hardware, camera_transform));
    uav_arm_system.reset(
        new UAVArmSystem(*tracker, drone_hardware, arm, config));
    logic_state_machine.reset(new PickPlaceStateMachine(
        boost::ref(*uav_arm_system), boost::cref(state_machine_config)));
    logic_state_machine->start();
    // Move to landed state
    logic_state_machine->process_event(InternalTransitionEvent());
  }

  ~PickPlaceStateMachineTests() {
    logic_state_machine->stop();
    uav_arm_system.reset();
    logic_state_machine.reset();
  }

protected:
  QuadSimulator drone_hardware;
  ArmSimulator arm;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;
  std::unique_ptr<SimpleTracker> tracker;
  std::unique_ptr<UAVArmSystem> uav_arm_system;
  std::unique_ptr<PickPlaceStateMachine> logic_state_machine;
  double goal_tolerance_position;

  void GoToHoverFromLanded() {
    drone_hardware.setBatteryPercent(100);
    logic_state_machine->process_event(be::Takeoff());
    // Powers on arm and folds it; Starts takeoff
    logic_state_machine->process_event(InternalTransitionEvent());
    // Completes takeoff and goes to hovering
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(PickPlaceStateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
  GoToHoverFromLanded();
}

/// \brief Test Arm folding during landing and takeoff
TEST_F(PickPlaceStateMachineTests, HoveringandLanding) {
  // Takeoff
  drone_hardware.setBatteryPercent(100);
  logic_state_machine->process_event(be::Takeoff());
  ASSERT_STREQ(pstate(*logic_state_machine), "ArmPreTakeoffFolding");
  // Powers on arm and folds it; Starts takeoff
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Takingoff");
  // Completes takeoff and goes to hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  logic_state_machine->process_event(be::Land());
  ASSERT_STREQ(pstate(*logic_state_machine), "ArmPreLandingFolding");
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Landing");
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}
///
/// \brief Test Pick Place
// Try Pick
TEST_F(PickPlaceStateMachineTests, PickPlace) {
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  // Check we are in PrePick state
  ASSERT_STREQ(pstate(*logic_state_machine), "RelativePoseVisualServoing");
  // Check UAV controller is active
  ASSERT_EQ(
      uav_arm_system
          ->getStatus<RelativePoseVisualServoingControllerDroneConnector>(),
      ControllerStatus::Active);
  ASSERT_EQ(uav_arm_system->getStatus<VisualServoingControllerArmConnector>(),
            ControllerStatus::NotEngaged);
  // Keep running the controller until its completed or timeout
  auto getUAVStatusRunControllers = [&]() {
    uav_arm_system->runActiveController(HardwareType::UAV);
    logic_state_machine->process_event(InternalTransitionEvent());
    return uav_arm_system->getStatus<
               RelativePoseVisualServoingControllerDroneConnector>() ==
           ControllerStatus::Active;
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(getUAVStatusRunControllers,
                                            std::chrono::seconds(5),
                                            std::chrono::milliseconds(0)));
  // Check we are in Pre-Pick
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  ASSERT_EQ(logic_state_machine->lastProcessedEventIndex(), typeid(Completed));
  auto getArmStatusRunControllers = [&]() {
    uav_arm_system->runActiveController(HardwareType::UAV);
    uav_arm_system->runActiveController(HardwareType::Arm);
    logic_state_machine->process_event(InternalTransitionEvent());
    return uav_arm_system->getStatus<VisualServoingControllerArmConnector>() ==
           ControllerStatus::Active;
  };
  // Run controllers again
  ASSERT_FALSE(test_utils::waitUntilFalse()(getArmStatusRunControllers,
                                            std::chrono::seconds(5)));
  // Check we are in Pick
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");
  ASSERT_EQ(logic_state_machine->lastProcessedEventIndex(), typeid(Completed));
}

TEST_F(PickPlaceStateMachineTests, Pick) {
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  // Check we are in PrePick state
  ASSERT_STREQ(pstate(*logic_state_machine), "RelativePoseVisualServoing");
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");

  arm.setGripperStatus(true);
  auto grip = [&]() {
    logic_state_machine->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(grip, std::chrono::seconds(5),
                                            std::chrono::milliseconds(20)));
  ASSERT_EQ(logic_state_machine->lastProcessedEventIndex(), typeid(Completed));
  // Check we are in hovering state
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(PickPlaceStateMachineTests, PickTimeout) {
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  // Check we are in PrePick state
  ASSERT_STREQ(pstate(*logic_state_machine), "RelativePoseVisualServoing");
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");

  arm.setGripperStatus(false);

  auto grip = [&]() {
    logic_state_machine->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(grip, std::chrono::seconds(5),
                                            std::chrono::milliseconds(20)));
  // Check we are in hovering state
  // \todo Matt This will not transition to hovering in the future, it will go
  // to ReachingGoal
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  ASSERT_EQ(logic_state_machine->lastProcessedEventIndex(), typeid(Reset));
}

TEST_F(PickPlaceStateMachineTests, PickWaitForGrip) {
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  // Check we are in PrePick state
  ASSERT_STREQ(pstate(*logic_state_machine), "RelativePoseVisualServoing");
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");

  // Initially not gripping
  arm.setGripperStatus(false);
  logic_state_machine->process_event(InternalTransitionEvent());
  this_thread::sleep_for(std::chrono::milliseconds(900));
  // Now gripping
  arm.setGripperStatus(true);

  auto grip = [&]() {
    logic_state_machine->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(grip, std::chrono::seconds(5),
                                            std::chrono::milliseconds(0)));
  ASSERT_EQ(logic_state_machine->lastProcessedEventIndex(), typeid(Completed));
  // Check we are in hovering state
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(PickPlaceStateMachineTests, PickGripTooLate) {
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  // Check we are in PrePick state
  ASSERT_STREQ(pstate(*logic_state_machine), "RelativePoseVisualServoing");
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");

  // Initially not gripping
  arm.setGripperStatus(false);
  logic_state_machine->process_event(InternalTransitionEvent());
  this_thread::sleep_for(std::chrono::milliseconds(1100));
  // Now gripping, but not enough time to complete grip before timeout
  arm.setGripperStatus(true);

  auto grip = [&]() {
    logic_state_machine->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(grip, std::chrono::seconds(5),
                                            std::chrono::milliseconds(0)));
  ASSERT_EQ(logic_state_machine->lastProcessedEventIndex(), typeid(Reset));
  // Check we are in hovering state
  // \todo Matt This will not transition to hovering in the future, it will go
  // to ReachingGoal
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

// Abort due to arm not enabled
TEST_F(PickPlaceStateMachineTests, PrePickPlaceArmAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Power off arm
  arm.sendCmd(ArmParser::Command::POWER_OFF);
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

// Abort due to arm not enabled
TEST_F(PickPlaceStateMachineTests, PickPlaceArmAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Go to pick
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");
  // Power off arm
  arm.sendCmd(ArmParser::Command::POWER_OFF);
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Abort due to tracking becoming invalid
TEST_F(PickPlaceStateMachineTests, PrePickPlaceInvalidTrackingAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Set tracker to invalid
  tracker->setTrackingIsValid(false);
  // Run active controllers
  for (int i = 0; i < 2; ++i) {
    uav_arm_system->runActiveController(HardwareType::UAV);
    uav_arm_system->runActiveController(HardwareType::Arm);
  }
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Abort due to tracking becoming invalid
TEST_F(PickPlaceStateMachineTests, PickPlaceInvalidTrackingAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Go to pick state
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");
  // Set tracker to invalid
  tracker->setTrackingIsValid(false);
  // Run active controllers
  for (int i = 0; i < 2; ++i) {
    uav_arm_system->runActiveController(HardwareType::UAV);
    uav_arm_system->runActiveController(HardwareType::Arm);
  }
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Manual rc abort
TEST_F(PickPlaceStateMachineTests, PrePickPlaceManualControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Disable SDK
  drone_hardware.flowControl(false);
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // And finally in manual control state
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
  // Enable SDK
  drone_hardware.flowControl(true);
  // Check we are back in hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Manual rc abort
TEST_F(PickPlaceStateMachineTests, PickPlaceManualControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  logic_state_machine->process_event(Completed());
  // Check we are in pre-pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Go to pick state
  logic_state_machine->process_event(Completed());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PickState");
  // Disable SDK
  drone_hardware.flowControl(false);
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // And finally in manual control state
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
  // Enable SDK
  drone_hardware.flowControl(true);
  // Check we are back in hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Manual control internal actions
TEST_F(PickPlaceStateMachineTests, PickPlaceManualControlInternalActions) {
  // Disable SDK
  drone_hardware.flowControl(false);
  // Move to manual control state
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check we are in manual control state
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
  // Enable arm
  uav_arm_system->power(true);
  // Process Poweroff
  logic_state_machine->process_event(arm_events::PowerOff());
  // Check arm is powered off
  ASSERT_FALSE(uav_arm_system->enabled());
  // Process Poweron
  logic_state_machine->process_event(arm_events::PowerOn());
  // Check arm is powered on
  ASSERT_TRUE(uav_arm_system->enabled());
  // Check command status is false
  ASSERT_FALSE(arm.getCommandStatus());
  // Check we can process fold
  logic_state_machine->process_event(arm_events::Fold());
  ASSERT_TRUE(arm.getCommandStatus());
  // Check we can process right fold
  logic_state_machine->process_event(arm_events::RightAngleFold());
  ASSERT_TRUE(arm.getCommandStatus());
  // Check we are still in Manual Control State
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
