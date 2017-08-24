#include <aerial_autonomy/state_machines/pick_place_state_machine.h>
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
    logic_state_machine.reset(
        new PickPlaceStateMachine(boost::ref(*uav_arm_system)));
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
  // Timeout for running controller
  int time_out = 5000;  // Milliseconds
  int time_period = 20; // Milliseconds
  int max_count = time_out / time_period;
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(pe::Pick());
  // Check we are in PrePick state
  ASSERT_STREQ(pstate(*logic_state_machine), "PrePickState");
  // Check controllers are active
  ASSERT_EQ(uav_arm_system->getStatus<VisualServoingControllerDroneConnector>(),
            ControllerStatus::Active);
  ASSERT_EQ(uav_arm_system->getStatus<VisualServoingControllerArmConnector>(),
            ControllerStatus::Active);
  // Keep running the controller until its completed or timeout
  int temp_count = 0;
  while (uav_arm_system->getStatus<VisualServoingControllerArmConnector>() ==
             ControllerStatus::Active &&
         ++temp_count < max_count) {
    uav_arm_system->runActiveController(HardwareType::UAV);
    uav_arm_system->runActiveController(HardwareType::Arm);
    logic_state_machine->process_event(InternalTransitionEvent());
    std::this_thread::sleep_for(std::chrono::milliseconds(time_period));
  }
  // Make sure controllers are aborted
  ASSERT_EQ(uav_arm_system->getActiveControllerStatus(HardwareType::Arm),
            ControllerStatus::NotEngaged);
  ASSERT_EQ(uav_arm_system->getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::NotEngaged);
  // Check we are back in Hovering state
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
