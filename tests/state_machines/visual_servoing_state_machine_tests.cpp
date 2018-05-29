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
      : drone_hardware(new QuadSimulator), goal_tolerance_position_(0.5) {
    drone_hardware->usePerfectTime();
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
    vs_position_tolerance->set_x(goal_tolerance_position_);
    vs_position_tolerance->set_y(goal_tolerance_position_);
    vs_position_tolerance->set_z(goal_tolerance_position_);

    // Configure position controller
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

    // Configure relative pose controller
    auto vel_controller_config =
        uav_vision_system_config
            ->mutable_rpyt_based_relative_pose_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config();
    vel_controller_config->mutable_goal_velocity_tolerance()->set_vx(0.1);
    vel_controller_config->mutable_goal_velocity_tolerance()->set_vy(0.1);
    vel_controller_config->mutable_goal_velocity_tolerance()->set_vz(0.1);
    auto rpyt_based_vel_controller_config =
        uav_vision_system_config
            ->mutable_rpyt_based_relative_pose_controller_config()
            ->mutable_rpyt_based_velocity_controller_config();
    rpyt_based_vel_controller_config->set_kp_xy(2.0);
    rpyt_based_vel_controller_config->set_ki_xy(0);
    rpyt_based_vel_controller_config->set_kp_z(2.0);
    rpyt_based_vel_controller_config->set_ki_z(0);
    auto vel_based_vs_controller_config =
        uav_vision_system_config
            ->mutable_rpyt_based_relative_pose_controller_config()
            ->mutable_velocity_based_relative_pose_controller_config()
            ->mutable_velocity_based_position_controller_config();
    vel_based_vs_controller_config->set_position_gain(0.5);
    vel_based_vs_controller_config->set_yaw_gain(0.5);
    vel_based_vs_controller_config->set_max_velocity(5.);
    vel_based_vs_controller_config->set_yaw_i_gain(0.0);
    vel_based_vs_controller_config->set_position_i_gain(0.0);
    vel_based_vs_controller_config->set_position_saturation_value(0.0);
    vel_based_vs_controller_config->set_yaw_saturation_value(0.0);
    auto relative_pose_vs_position_tolerance =
        vel_based_vs_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_based_vs_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(0.1);
    relative_pose_vs_position_tolerance->set_x(goal_tolerance_position_);
    relative_pose_vs_position_tolerance->set_y(goal_tolerance_position_);
    relative_pose_vs_position_tolerance->set_z(goal_tolerance_position_);
    // Add pose goal
    auto pose_goal =
        state_machine_config.mutable_visual_servoing_state_machine_config()
            ->add_relative_pose_goals();
    auto pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(-1);
    pose_goal_position->set_y(0);
    pose_goal_position->set_z(0);
    pose_goal->set_yaw(0);

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
    drone_hardware->usePerfectTime();
  }

  void runActiveControllerToConvergence() {
    auto getUAVStatusRunControllers = [&]() {
      uav_system->runActiveController(ControllerGroup::UAV);
      return uav_system->getActiveControllerStatus(ControllerGroup::UAV) ==
             ControllerStatus::Completed;
    };
    ASSERT_TRUE(test_utils::waitUntilTrue()(getUAVStatusRunControllers,
                                            std::chrono::seconds(1),
                                            std::chrono::milliseconds(0)));
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
  double goal_tolerance_position_;

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
  ASSERT_EQ(uav_system->getStatus<RPYTRelativePoseVisualServoingConnector>(),
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
