#include <aerial_autonomy/state_machines/mpc_state_machine.h>
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
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

/**
* @brief Namespace for mpc events
*/
namespace me = mpc_events;

class MPCStateMachineTests : public ::testing::Test {
public:
  MPCStateMachineTests()
      : drone_hardware_(new QuadSimulator), arm_(new ArmSimulator),
        goal_tolerance_position_(0.1) {
    auto mpc_state_machine_config =
        state_machine_config_.mutable_mpc_state_machine_config();
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
    auto spiral_reference =
        mpc_state_machine_config->mutable_spiral_reference();
    spiral_reference->set_radiusx(1.0);
    spiral_reference->set_radiusy(1.0);
    spiral_reference->set_frequency(0.01);
    // Waypoint
    auto waypoint_ref = mpc_state_machine_config->mutable_waypoint_reference();
    waypoint_ref->set_yaw(0.2);
    auto position_ref = waypoint_ref->mutable_position();
    position_ref->set_x(1.0);
    position_ref->set_y(0.5);
    position_ref->set_z(0.4);

    // Position controller params
    auto pos_controller_config =
        config_.mutable_rpyt_based_position_controller_config()
            ->mutable_velocity_based_position_controller_config();
    pos_controller_config->set_position_gain(1.);
    pos_controller_config->set_yaw_gain(1.);
    pos_controller_config->set_max_velocity(2.);
    pos_controller_config->set_max_yaw_rate(5.);
    auto goal_position_tolerance =
        pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    pos_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(0.1);
    goal_position_tolerance->set_x(goal_tolerance_position_);
    goal_position_tolerance->set_y(goal_tolerance_position_);
    goal_position_tolerance->set_z(goal_tolerance_position_);
    auto rpyt_vel_controller_tol =
        config_.mutable_rpyt_based_position_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    rpyt_vel_controller_tol->set_vx(0.1);
    rpyt_vel_controller_tol->set_vy(0.1);
    rpyt_vel_controller_tol->set_vz(0.1);
    // Disable Pose sensor since it depends on ros
    auto uav_arm_system_config = config_.mutable_uav_vision_system_config()
                                     ->mutable_uav_arm_system_config();
    uav_arm_system_config->set_visualize_mpc_trajectories(false);
    // Fill MPC Config
    test_utils::fillMPCConfig(config_);

    tf::Transform camera_transform;
    camera_transform.setIdentity();
    tracker_.reset(new SimpleTracker(*drone_hardware_, camera_transform));
    uav_arm_system_.reset(new UAVArmSystem(
        config_, std::dynamic_pointer_cast<BaseTracker>(tracker_),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware_),
        std::dynamic_pointer_cast<ArmSimulator>(arm_)));
    logic_state_machine_.reset(new MPCStateMachine(
        boost::ref(*uav_arm_system_), boost::cref(state_machine_config_)));
    logic_state_machine_->start();
    // Use simulated time for drone hardware
    drone_hardware_->usePerfectTime();
    uav_arm_system_->usePerfectMPCTime(0.02);
    // Set delay to small value:
    drone_hardware_->set_delay_send_time(0.02);
    // Moves the state to manual control state
    logic_state_machine_->process_event(InternalTransitionEvent());
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
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
  }

  ~MPCStateMachineTests() {
    logic_state_machine_->stop();
    uav_arm_system_.reset();
    logic_state_machine_.reset();
  }

protected:
  std::shared_ptr<QuadSimulator> drone_hardware_;
  std::shared_ptr<ArmSimulator> arm_;
  UAVSystemConfig config_;
  BaseStateMachineConfig state_machine_config_;
  std::shared_ptr<SimpleTracker> tracker_;
  std::unique_ptr<UAVArmSystem> uav_arm_system_;
  std::unique_ptr<MPCStateMachine> logic_state_machine_;
  double goal_tolerance_position_;

  ///\todo Move these functions into a common class as they are more generic
  /// than this test fixture
  template <class EventT> void testManualControlAbort() {
    // First takeoff
    GoToHoverFromLanded();
    logic_state_machine_->process_event(EventT());
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STRNE(pstate(*logic_state_machine_), "Hovering");
    // Disable SDK
    drone_hardware_->flowControl(false);
    // Check we are in Hovering
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
    // And finally in manual control state
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "ManualControlArmState");
    // Enable SDK
    drone_hardware_->flowControl(true);
    // Check we are back in hovering
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
  }

  template <class EventT> void testArmOffAbort() {
    // First takeoff
    GoToHoverFromLanded();
    logic_state_machine_->process_event(EventT());
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STRNE(pstate(*logic_state_machine_), "Hovering");
    // Power off arm
    arm_->sendCmd(ArmParser::Command::POWER_OFF);
    // Check we are in Hovering
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
  }
  void runActiveControllerToConvergence() {
    auto getStatusRunControllers = [&]() {
      uav_arm_system_->runActiveController(ControllerGroup::UAV);
      uav_arm_system_->runActiveController(ControllerGroup::Arm);
      auto data = uav_arm_system_->getUAVData();
      return (uav_arm_system_->getActiveControllerStatus(
                  ControllerGroup::UAV) == ControllerStatus::Active);
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                              std::chrono::seconds(20),
                                              std::chrono::milliseconds(0)));
  }

  template <class EventT> void testMPCTracking() {
    GoToHoverFromLanded();
    logic_state_machine_->process_event(EventT());
    ASSERT_STREQ(pstate(*logic_state_machine_), "MPCState");
    // Keep running controller until converged
    runActiveControllerToConvergence();
    auto status =
        uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV);
    ASSERT_EQ(status, ControllerStatus::Completed) << status.statusAsText();
  }

  void GoToHoverFromLanded() {
    drone_hardware_->setBatteryPercent(100);
    drone_hardware_->takeoff();
    uav_arm_system_->power(true);
    uav_arm_system_->foldArm();
    logic_state_machine_->process_event(be::Takeoff());
  }
};

TEST_F(MPCStateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine_), "ManualControlArmState");
  GoToHoverFromLanded();
  ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
}

/// \brief Test Arm folding during landing and takeoff
/// \brief Test Pick Place
TEST_F(MPCStateMachineTests, MPCSpiralTracking) {
  testMPCTracking<me::MPCSpiralTrack>();
}

TEST_F(MPCStateMachineTests, MPCWaypointTracking) {
  testMPCTracking<me::MPCWaypointTrack>();
}

TEST_F(MPCStateMachineTests, MPCAbort) {
  GoToHoverFromLanded();
  logic_state_machine_->process_event(me::MPCWaypointTrack());
  ASSERT_STREQ(pstate(*logic_state_machine_), "MPCState");
  logic_state_machine_->process_event(be::Abort());
  logic_state_machine_->process_event(InternalTransitionEvent());
  auto status =
      uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV);
  ASSERT_EQ(status, ControllerStatus::NotEngaged) << status.statusAsText();
  ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
}
// Test gohome
TEST_F(MPCStateMachineTests, GoHome) {
  // First takeoff
  GoToHoverFromLanded();
  logic_state_machine_->process_event(me::GoHome());
  // Check we are still in Hovering since we did not save home location
  ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
  // Save home location now
  uav_arm_system_->setHomeLocation();
  // Send drone somewhere
  geometry_msgs::Vector3 desired_position;
  desired_position.x = desired_position.y = desired_position.z = 3.0;
  drone_hardware_->cmdwaypoint(desired_position);
  // Try getting back to home
  logic_state_machine_->process_event(me::GoHome());
  // Check we are reaching home position
  ASSERT_STREQ(pstate(*logic_state_machine_), "ReachingGoal");
  // Run active controller
  runActiveControllerToConvergence();
  // Check we are at home location
  auto uav_data = uav_arm_system_->getUAVData();
  ASSERT_NEAR(uav_data.localpos.x, 0.0, 0.1);
  ASSERT_NEAR(uav_data.localpos.y, 0.0, 0.1);
  ASSERT_NEAR(uav_data.localpos.z, 0.5, 0.1);
  // Process status to get out of reaching goal state
  logic_state_machine_->process_event(InternalTransitionEvent());
  // Check we are back in hovering
  ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
