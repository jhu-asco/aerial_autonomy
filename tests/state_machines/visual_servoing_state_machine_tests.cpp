#include <aerial_autonomy/state_machines/visual_servoing_state_machine.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
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
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

class VisualServoingStateMachineTests : public ::testing::Test {
public:
  VisualServoingStateMachineTests() : goal_tolerance_position(0.5) {
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
    for (int i = 0; i < 6; ++i) {
      uav_vision_system_config->add_camera_transform(0.0);
    }
    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    auto depth_config =
        uav_vision_system_config
            ->mutable_constant_heading_depth_controller_config();
    depth_config->set_radial_gain(0.5);
    auto position_tolerance = config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.5);
    position_tolerance->set_y(0.5);
    position_tolerance->set_z(0.5);
    auto vs_position_tolerance =
        depth_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vs_position_tolerance->set_x(goal_tolerance_position);
    vs_position_tolerance->set_y(goal_tolerance_position);
    vs_position_tolerance->set_z(goal_tolerance_position);
    tf::Transform camera_transform = math::getTransformFromVector(
        uav_vision_system_config->camera_transform());
    tracker.reset(new SimpleTracker(drone_hardware, camera_transform));
    uav_system.reset(new UAVVisionSystem(*tracker, drone_hardware, config));
    logic_state_machine.reset(
        new VisualServoingStateMachine(boost::ref(*uav_system)));
    logic_state_machine->start();
    // Move to landed state
    logic_state_machine->process_event(InternalTransitionEvent());
  }

  ~VisualServoingStateMachineTests() {
    logic_state_machine->stop();
    uav_system.reset();
    logic_state_machine.reset();
  }

protected:
  QuadSimulator drone_hardware;
  UAVSystemConfig config;
  std::unique_ptr<SimpleTracker> tracker;
  std::unique_ptr<UAVVisionSystem> uav_system;
  std::unique_ptr<VisualServoingStateMachine> logic_state_machine;
  double goal_tolerance_position;

  void GoToHoverFromLanded() {
    drone_hardware.setBatteryPercent(100);
    logic_state_machine->process_event(be::Takeoff());
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(VisualServoingStateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

/// \brief Test Visual servoing related events
TEST_F(VisualServoingStateMachineTests, VisualServoing) {
  int time_out = 5000;  // Milliseconds
  int time_period = 20; // Milliseconds
  int max_count = time_out / time_period;
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(vse::TrackROI());
  // Check we are in visual servoing state
  ASSERT_STREQ(pstate(*logic_state_machine), "VisualServoing");
  // Check controller status
  ASSERT_EQ(uav_system->getStatus<VisualServoingControllerDroneConnector>(),
            ControllerStatus::Active);
  // Keep running the controller until its completed
  int temp_count = 0;
  while (uav_system->getStatus<VisualServoingControllerDroneConnector>() ==
             ControllerStatus::Active &&
         ++temp_count < max_count) {
    uav_system->runActiveController(HardwareType::UAV);
    logic_state_machine->process_event(InternalTransitionEvent());
    std::this_thread::sleep_for(std::chrono::milliseconds(time_period));
  }
  // Finally check we are back in hovering
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Abort Visual servoing
TEST_F(VisualServoingStateMachineTests, VisualServoingAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(vse::TrackROI());
  // Check we are in visual servoing state
  ASSERT_STREQ(pstate(*logic_state_machine), "VisualServoing");
  // Abort
  logic_state_machine->process_event(be::Abort());
  // Check we are at same place where we started
  auto uav_data = uav_system->getUAVData();
  ASSERT_EQ(uav_data.localpos.x, 0.0);
  ASSERT_EQ(uav_data.localpos.y, 0.0);
  ASSERT_EQ(uav_data.localpos.z, 0.5);
  // Check we are back in hovering
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
// Manual rc abort
TEST_F(VisualServoingStateMachineTests, VisualServoingManualControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Set goal for simple tracker
  Position roi_goal(2, 0, 0.5);
  tracker->setTargetPositionGlobalFrame(roi_goal);
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(vse::TrackROI());
  // Check we are in visual servoing state
  ASSERT_STREQ(pstate(*logic_state_machine), "VisualServoing");
  // Disable SDK
  drone_hardware.flowControl(false);
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // And finally in manual control state
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
  // Enable SDK
  drone_hardware.flowControl(true);
  // CHeck we are back in hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
///
/// \brief Test GoHome related events
TEST_F(VisualServoingStateMachineTests, GoHome) {
  // First takeoff
  GoToHoverFromLanded();
  // Initialize event to vse::TrackROI
  logic_state_machine->process_event(vse::GoHome());
  // Check we are still in Hovering since we did not save home location
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Save home location now
  uav_system->setHomeLocation();
  // Send drone somewhere
  geometry_msgs::Vector3 desired_position;
  desired_position.x = desired_position.y = desired_position.z = 3.0;
  drone_hardware.cmdwaypoint(desired_position);
  // Try getting back to home
  logic_state_machine->process_event(vse::GoHome());
  // Check we are reaching home position
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
  // Run active controller
  uav_system->runActiveController(HardwareType::UAV);
  // Check we are at home location
  auto uav_data = uav_system->getUAVData();
  ASSERT_EQ(uav_data.localpos.x, 0.0);
  ASSERT_EQ(uav_data.localpos.y, 0.0);
  ASSERT_EQ(uav_data.localpos.z, 0.5);
  // Update status
  uav_system->runActiveController(HardwareType::UAV);
  // Process status to get out of reaching goal state
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check we are back in hovering
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
