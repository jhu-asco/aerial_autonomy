#include <aerial_autonomy/state_machines/visual_servoing_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
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
  VisualServoingStateMachineTests()
      : drone_hardware(new QuadSimulator), goal_tolerance_position(0.5) {
    auto uav_vision_system_config = config.mutable_uav_vision_system_config();
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

    // Configure position controller
    auto vel_based_pos_controller_config =
        config.mutable_velocity_based_position_controller_config();
    vel_based_pos_controller_config->set_position_gain(50.);
    vel_based_pos_controller_config->set_yaw_gain(50.);
    auto vel_based_pos_controller_tol =
        vel_based_pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_based_pos_controller_tol->set_x(0.1);
    vel_based_pos_controller_tol->set_y(0.1);
    vel_based_pos_controller_tol->set_z(0.1);
    vel_based_pos_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(0.1);

    tf::Transform camera_transform = conversions::protoTransformToTf(
        uav_vision_system_config->camera_transform());
    tracker.reset(new SimpleTracker(*drone_hardware, camera_transform));
    uav_system.reset(new UAVVisionSystem(
        config, std::dynamic_pointer_cast<BaseTracker>(tracker),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware)));
    logic_state_machine.reset(new VisualServoingStateMachine(
        boost::ref(*uav_system), boost::cref(state_machine_config)));
    logic_state_machine->start();
    // Move to landed state
    logic_state_machine->process_event(InternalTransitionEvent());
  }

  void runActiveControllerToConvergence() {
    auto getUAVStatusRunControllers = [&]() {
      uav_system->runActiveController(HardwareType::UAV);
      return uav_system->getActiveControllerStatus(HardwareType::UAV) ==
             ControllerStatus::Completed;
    };
    ASSERT_TRUE(test_utils::waitUntilTrue()(getUAVStatusRunControllers,
                                            std::chrono::seconds(5),
                                            std::chrono::milliseconds(0)));
  }

  ~VisualServoingStateMachineTests() {
    logic_state_machine->stop();
    uav_system.reset();
    logic_state_machine.reset();
  }

protected:
  std::shared_ptr<QuadSimulator> drone_hardware;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;
  std::shared_ptr<SimpleTracker> tracker;
  std::unique_ptr<UAVVisionSystem> uav_system;
  std::unique_ptr<VisualServoingStateMachine> logic_state_machine;
  double goal_tolerance_position;

  void GoToHoverFromLanded() {
    drone_hardware->setBatteryPercent(100);
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

  runActiveControllerToConvergence();

  logic_state_machine->process_event(InternalTransitionEvent());
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
  drone_hardware->flowControl(false);
  // Check we are in Hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // And finally in manual control state
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
  // Enable SDK
  drone_hardware->flowControl(true);
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
  drone_hardware->cmdwaypoint(desired_position);
  // Try getting back to home
  logic_state_machine->process_event(vse::GoHome());
  // Check we are reaching home position
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
  // Run active controller
  runActiveControllerToConvergence();
  // Check we are at home location
  auto uav_data = uav_system->getUAVData();
  ASSERT_NEAR(uav_data.localpos.x, 0.0, 0.1);
  ASSERT_NEAR(uav_data.localpos.y, 0.0, 0.1);
  ASSERT_NEAR(uav_data.localpos.z, 0.5, 0.1);
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
