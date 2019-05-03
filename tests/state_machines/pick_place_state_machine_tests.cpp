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
  PickPlaceStateMachineTests()
      : drone_hardware_(new QuadSimulator), arm_(new ArmSimulator),
        goal_tolerance_position_(1.0), grip_timeout_(25000),
        grip_duration_(10) {
    auto vision_state_machine_config =
        state_machine_config_.mutable_visual_servoing_state_machine_config();
    auto pick_state_machine_config =
        vision_state_machine_config->mutable_pick_place_state_machine_config();
    auto grip_config = pick_state_machine_config->mutable_grip_config();
    // Set visual servoing controller
    vision_state_machine_config->set_connector_type(
        VisualServoingStateMachineConfig::RPYTPose);
    // Configure place groups
    uint32_t dest1 = 15;
    uint32_t dest2 = 16;
    auto place_group1 = pick_state_machine_config->add_place_groups();
    place_group1->set_destination_id(dest1);
    place_group1->add_object_ids(0);
    place_group1->add_object_ids(1);
    auto place_group2 = pick_state_machine_config->add_place_groups();
    place_group2->set_destination_id(dest2);
    place_group2->add_object_ids(2);
    place_group2->add_object_ids(3);

    config_.mutable_rpyt_reference_connector_config()
        ->set_use_perfect_time_diff(true);
    auto uav_vision_system_config = config_.mutable_uav_vision_system_config();
    uav_vision_system_config->set_desired_visual_servoing_distance(1.0);
    auto uav_arm_system_config =
        uav_vision_system_config->mutable_uav_arm_system_config();
    auto depth_config =
        uav_vision_system_config
            ->mutable_constant_heading_depth_controller_config();
    depth_config->set_radial_gain(0.5);
    auto position_tolerance = depth_config->mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(goal_tolerance_position_);
    position_tolerance->set_y(goal_tolerance_position_);
    position_tolerance->set_z(goal_tolerance_position_);
    // Grip parameters
    grip_config->set_grip_timeout(grip_timeout_);
    grip_config->set_grip_duration(grip_duration_);

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
    rpyt_vel_controller_tol->set_vx(0.5);
    rpyt_vel_controller_tol->set_vy(0.5);
    rpyt_vel_controller_tol->set_vz(0.5);

    // Relative visual servoing controller params
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
    // Set delay to small value:
    drone_hardware_->set_delay_send_time(0.02);

    // Arm goals
    pick_state_machine_config->add_arm_goal_transform();
    pick_state_machine_config->add_arm_goal_transform();

    // Relative marker goal for pick
    auto pose_goal = vision_state_machine_config->add_relative_pose_goals();
    auto pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(1);
    pose_goal_position->set_y(1);
    pose_goal_position->set_z(2);
    pose_goal->set_yaw(0);
    // Relative marker goal for place
    vision_state_machine_config->add_relative_pose_goals();
    // Pre-Pick
    auto pre_pick_goal = vision_state_machine_config->add_relative_pose_goals();
    auto pre_pick_goal_position = pre_pick_goal->mutable_position();
    pre_pick_goal_position->set_x(0.5);
    pre_pick_goal_position->set_y(1);
    pre_pick_goal_position->set_z(2);
    pre_pick_goal->set_yaw(0);
    // Pre-Place
    auto pre_place_goal =
        vision_state_machine_config->add_relative_pose_goals();
    auto pre_place_goal_position = pre_pick_goal->mutable_position();
    pre_place_goal_position->set_x(1);
    pre_place_goal_position->set_y(1);
    pre_place_goal_position->set_z(2);
    pre_place_goal->set_yaw(0);

    auto waypoint_config =
        pick_state_machine_config->mutable_following_waypoint_sequence_config();
    auto waypoint_position_controller_config =
        waypoint_config->mutable_position_controller_config();
    *(waypoint_position_controller_config) =
        *(pos_controller_config->mutable_position_controller_config());
    // Post-pick waypoints
    for (int i = 0; i < 2; i++) {
      auto pose_goal = waypoint_config->add_way_points();
      auto pose_goal_position = pose_goal->mutable_position();
      pose_goal_position->set_x(-i);
      pose_goal_position->set_y(0);
      pose_goal_position->set_z(1);
      pose_goal->set_yaw(0);
    }

    // Post-place waypoints
    for (int i = 0; i < 2; i++) {
      auto pose_goal = waypoint_config->add_way_points();
      auto pose_goal_position = pose_goal->mutable_position();
      pose_goal_position->set_x(2);
      pose_goal_position->set_y(0);
      pose_goal_position->set_z(0.5);
      pose_goal->set_yaw(M_PI);
    }

    // Arm controller params
    auto arm_position_tolerance =
        uav_arm_system_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    arm_position_tolerance->set_x(goal_tolerance_position_);
    arm_position_tolerance->set_y(goal_tolerance_position_);
    arm_position_tolerance->set_z(goal_tolerance_position_);
    // Fill quad mpc config
    auto mpc_config = config_.mutable_quad_mpc_controller_config();
    test_utils::fillQuadMPCConfig(*mpc_config);
    // Fill exponential config
    auto particle_reference_config =
        uav_vision_system_config->mutable_particle_reference_config();
    particle_reference_config->set_max_velocity(1.0);
    // Fill MPC Config
    test_utils::fillMPCConfig(config_);

    tf::Transform camera_transform = conversions::protoTransformToTf(
        uav_vision_system_config->camera_transform());
    tracker_.reset(new SimpleTracker(*drone_hardware_, camera_transform));
    uav_arm_system_.reset(new UAVArmSystem(
        config_, std::dynamic_pointer_cast<BaseTracker>(tracker_),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware_),
        std::dynamic_pointer_cast<ArmSimulator>(arm_)));
    logic_state_machine_.reset(new PickPlaceStateMachine(
        boost::ref(*uav_arm_system_), boost::cref(state_machine_config_)));
    logic_state_machine_->start();
    // Move to landed state
    logic_state_machine_->process_event(InternalTransitionEvent());

    // Targets used in pick place
    // Target objects
    targets_[0] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 0, 0.5));
    targets_[1] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(3, 0, 0.5));
    targets_[2] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(4, 0, 0.5));
    targets_[3] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(5, 0, 0.5));
    // Place goals
    targets_[dest1] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-3, 0, 0.5));
    targets_[dest2] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-3, 0, 1.5));
    tracker_->setTargetPosesGlobalFrame(targets_);
    // Use simulated time for drone hardware
    drone_hardware_->usePerfectTime();
  }

  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("rpyt_reference_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("rpyt_reference_connector");
    Log::instance().addDataStream(data_config);
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
    data_config.set_stream_id("quad_mpc_state_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("ddp_quad_mpc_controller");
    Log::instance().addDataStream(data_config);
  }

  ~PickPlaceStateMachineTests() {
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
  std::unique_ptr<PickPlaceStateMachine> logic_state_machine_;
  double goal_tolerance_position_;
  uint32_t grip_timeout_;
  uint32_t grip_duration_;
  std::unordered_map<uint32_t, tf::Transform> targets_;

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

  void GoToHoverFromLanded() {
    drone_hardware_->setBatteryPercent(100);
    logic_state_machine_->process_event(be::Takeoff());
    // Powers on arm and folds it; Starts takeoff
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Completes takeoff and goes to hovering
    logic_state_machine_->process_event(InternalTransitionEvent());
  }

  void PickPlace(uint32_t expected_place_target) {
    // Check we are waiting for pick
    ASSERT_STREQ(pstate(*logic_state_machine_), "WaitingForPick");
    // Check in PickState
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "PrePickState");
    // Check UAV and arm controllers are active
    ASSERT_EQ(
        uav_arm_system_->getStatus<RPYTRelativePoseVisualServoingConnector>(),
        ControllerStatus::Active);
    ASSERT_EQ(uav_arm_system_->getStatus<BuiltInPoseControllerArmConnector>(),
              ControllerStatus::Active);
    // Keep running the controller until its completed or timeout
    auto getStatusRunControllers = [&]() {
      uav_arm_system_->runActiveController(ControllerGroup::UAV);
      uav_arm_system_->runActiveController(ControllerGroup::Arm);
      return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
                 ControllerStatus::Active ||
             uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
                 ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    // Grip object for required duration
    arm_->setGripperStatus(true);
    logic_state_machine_->process_event(InternalTransitionEvent());
    this_thread::sleep_for(std::chrono::milliseconds(grip_duration_ + 2));
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Check we are in Post-Pick
    ASSERT_STREQ(pstate(*logic_state_machine_), "ReachingPostPickWaypoint");
    ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(),
              typeid(ObjectId));
    // Run controllers through one waypoint
    logic_state_machine_->process_event(InternalTransitionEvent());
    auto getStatusRunControllersv2 = [&]() {
      uav_arm_system_->runActiveController(ControllerGroup::UAV);
      uav_arm_system_->runActiveController(ControllerGroup::Arm);
      return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
             ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllersv2,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Check we are in Pre-Place
    ASSERT_STREQ(pstate(*logic_state_machine_), "PrePlaceState");
    LOG(INFO) << "In Pre-Place State!!";
    ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(),
              typeid(ObjectId));
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllersv2,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Check we are in Pre-Place
    ASSERT_STREQ(pstate(*logic_state_machine_), "PlaceState");
    ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(),
              typeid(Completed));
    LOG(INFO) << "In Place State!!";
    // Place the object in the correct spot
    uint32_t actual_place_target;
    ASSERT_TRUE(uav_arm_system_->getTrackingVectorId(actual_place_target));
    ASSERT_EQ(actual_place_target, expected_place_target);
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Check Place completed with ungrip
    ASSERT_STREQ(pstate(*logic_state_machine_), "ReachingPostPlaceWaypoint");
    ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(),
              typeid(Completed));
    ASSERT_FALSE(arm_->getGripperValue());
    // Run controllers through two waypoints
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllersv2,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllersv2,
                                              std::chrono::seconds(25),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Check we are back to waiting for pick
    ASSERT_STREQ(pstate(*logic_state_machine_), "WaitingForPick");
    ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(),
              typeid(Completed));
  }
};

TEST_F(PickPlaceStateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine_), "Landed");
  GoToHoverFromLanded();
}

/// \brief Test Arm folding during landing and takeoff
TEST_F(PickPlaceStateMachineTests, HoveringandLanding) {
  // Takeoff
  drone_hardware_->setBatteryPercent(100);
  logic_state_machine_->process_event(be::Takeoff());
  ASSERT_STREQ(pstate(*logic_state_machine_), "ArmPreTakeoffFolding");
  // Powers on arm and folds it; Starts takeoff
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "Takingoff");
  // Completes takeoff and goes to hovering
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
  logic_state_machine_->process_event(be::Land());
  ASSERT_STREQ(pstate(*logic_state_machine_), "ArmPreLandingFolding");
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "Landing");
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "Landed");
}
///
/// \brief Test Pick Place
// Try full pick-and-place
TEST_F(PickPlaceStateMachineTests, PickPlace) {
  auto pick_state_machine_config =
      state_machine_config_.visual_servoing_state_machine_config()
          .pick_place_state_machine_config();
  GoToHoverFromLanded();
  // Start Pick
  logic_state_machine_->process_event(pe::Pick());
  PickPlace(pick_state_machine_config.place_groups().Get(0).destination_id());
  // Remove targets for first destination
  targets_.erase(0);
  targets_.erase(1);
  tracker_->setTargetPosesGlobalFrame(targets_);
  // Repeat pick place for other targets
  PickPlace(pick_state_machine_config.place_groups().Get(1).destination_id());
}

TEST_F(PickPlaceStateMachineTests, PickTimeout) {
  GoToHoverFromLanded();
  // Start Pick
  logic_state_machine_->process_event(pe::Pick());
  // Check we are waiting for pick
  ASSERT_STREQ(pstate(*logic_state_machine_), "WaitingForPick");
  // Check in PickState
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PrePickState");
  // Check UAV and arm controllers are active
  ASSERT_EQ(
      uav_arm_system_->getStatus<RPYTRelativePoseVisualServoingConnector>(),
      ControllerStatus::Active);
  ASSERT_EQ(uav_arm_system_->getStatus<BuiltInPoseControllerArmConnector>(),
            ControllerStatus::Active);
  // Keep running the controller until its completed or timeout
  auto getStatusRunControllers = [&]() {
    uav_arm_system_->runActiveController(ControllerGroup::UAV);
    uav_arm_system_->runActiveController(ControllerGroup::Arm);
    return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
               ControllerStatus::Active ||
           uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
               ControllerStatus::Active;
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                            std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                            std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  // Grip has failed
  arm_->setGripperStatus(false);

  auto grip = [&]() {
    logic_state_machine_->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine_) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(
      grip, std::chrono::milliseconds(grip_timeout_ + 2),
      std::chrono::milliseconds(0)));
  // Check we are resetting
  ASSERT_STREQ(pstate(*logic_state_machine_), "ResetVisualServoing");
  ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(), typeid(Reset));
}

TEST_F(PickPlaceStateMachineTests, PickWaitForGrip) {
  GoToHoverFromLanded();
  // Start Pick
  logic_state_machine_->process_event(pe::Pick());
  // Check we are waiting for pick
  ASSERT_STREQ(pstate(*logic_state_machine_), "WaitingForPick");
  // Check in PickState
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PrePickState");
  // Keep running the controller until its completed or timeout
  auto getStatusRunControllers = [&]() {
    uav_arm_system_->runActiveController(ControllerGroup::UAV);
    uav_arm_system_->runActiveController(ControllerGroup::Arm);
    return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
               ControllerStatus::Active ||
           uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
               ControllerStatus::Active;
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                            std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                            std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  VLOG(1) << "State: " << pstate(*logic_state_machine_);
  // Initially not gripping
  arm_->setGripperStatus(false);
  logic_state_machine_->process_event(InternalTransitionEvent());
  this_thread::sleep_for(std::chrono::milliseconds(grip_duration_ - 2));
  // Now gripping
  arm_->setGripperStatus(true);

  auto grip = [&]() {
    logic_state_machine_->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine_) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(grip, std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  std::cout << "Last processed event: "
            << (logic_state_machine_->lastProcessedEventIndex()).name()
            << std::endl;
  ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(), typeid(ObjectId));
  // Check we are in hovering state
  ASSERT_STREQ(pstate(*logic_state_machine_), "ReachingPostPickWaypoint");
}

TEST_F(PickPlaceStateMachineTests, PickGripTooLate) {
  GoToHoverFromLanded();
  // Start Pick
  logic_state_machine_->process_event(pe::Pick());
  // Check we are waiting for pick
  ASSERT_STREQ(pstate(*logic_state_machine_), "WaitingForPick");
  // Check in PickState
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PrePickState");
  // Keep running the controller until its completed or timeout
  auto getStatusRunControllers = [&]() {
    uav_arm_system_->runActiveController(ControllerGroup::UAV);
    uav_arm_system_->runActiveController(ControllerGroup::Arm);
    return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
               ControllerStatus::Active ||
           uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
               ControllerStatus::Active;
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                            std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  // Initially not gripping
  arm_->setGripperStatus(false);
  logic_state_machine_->process_event(InternalTransitionEvent());
  this_thread::sleep_for(
      std::chrono::milliseconds(grip_timeout_ - grip_duration_ + 2));
  // Now gripping, but not enough time to complete grip before timeout
  arm_->setGripperStatus(true);

  auto grip = [&]() {
    logic_state_machine_->process_event(InternalTransitionEvent());
    return pstate(*logic_state_machine_) == std::string("PickState");
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(
      grip, std::chrono::milliseconds(grip_duration_),
      std::chrono::milliseconds(0)));
  ASSERT_STREQ(pstate(*logic_state_machine_), "ResetVisualServoing");
  ASSERT_EQ(logic_state_machine_->lastProcessedEventIndex(), typeid(Reset));
}

// Abort due to arm not enabled
TEST_F(PickPlaceStateMachineTests, PickArmAbort) {
  testArmOffAbort<pe::Pick>();
}

// Abort due to tracking becoming invalid
TEST_F(PickPlaceStateMachineTests, PickPlaceInvalidTrackingAbort) {
  // First takeoff
  GoToHoverFromLanded();
  logic_state_machine_->process_event(pe::Pick());
  logic_state_machine_->process_event(InternalTransitionEvent());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine_), "PrePickState");
  // Set tracker to invalid
  tracker_->setTrackingIsValid(false);
  // Run active controllers
  for (int i = 0; i < 2; ++i) {
    uav_arm_system_->runActiveController(ControllerGroup::HighLevel);
    uav_arm_system_->runActiveController(ControllerGroup::UAV);
    uav_arm_system_->runActiveController(ControllerGroup::Arm);
  }
  // Check we are resetting
  arm_->setGripperStatus(false);
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "ResetVisualServoing");
}
// Abort due to tracking becoming invalid
TEST_F(PickPlaceStateMachineTests, PickPlaceInvalidTrackingGripping) {
  // First takeoff
  GoToHoverFromLanded();
  logic_state_machine_->process_event(pe::Pick());
  logic_state_machine_->process_event(InternalTransitionEvent());
  // Check we are in pick state
  ASSERT_STREQ(pstate(*logic_state_machine_), "PrePickState");
  // Keep running the controller until its completed or timeout
  auto getStatusRunControllers = [&]() {
    uav_arm_system_->runActiveController(ControllerGroup::UAV);
    uav_arm_system_->runActiveController(ControllerGroup::Arm);
    return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
               ControllerStatus::Active ||
           uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
               ControllerStatus::Active;
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                            std::chrono::seconds(25),
                                            std::chrono::milliseconds(0)));
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  // Set tracker to invalid
  tracker_->setTrackingIsValid(false);
  // Run active controllers
  for (int i = 0; i < 2; ++i) {
    uav_arm_system_->runActiveController(ControllerGroup::HighLevel);
    uav_arm_system_->runActiveController(ControllerGroup::UAV);
    uav_arm_system_->runActiveController(ControllerGroup::Arm);
  }
  // Check we are continuing to pick and controller is aborted
  arm_->setGripperStatus(true);
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  ASSERT_EQ(ControllerStatus::NotEngaged,
            uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV));

  // If gripping becomes false, should now abort
  arm_->setGripperStatus(false);
  logic_state_machine_->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine_), "ResetVisualServoing");
}
// Manual rc abort
TEST_F(PickPlaceStateMachineTests, PickManualControlAbort) {
  testManualControlAbort<pe::Pick>();
}
// Manual control internal actions
TEST_F(PickPlaceStateMachineTests, PickPlaceManualControlInternalActions) {
  // Disable SDK
  drone_hardware_->flowControl(false);
  // Move to manual control state
  logic_state_machine_->process_event(InternalTransitionEvent());
  // Check we are in manual control state
  ASSERT_STREQ(pstate(*logic_state_machine_), "ManualControlArmState");
  // Enable arm
  uav_arm_system_->power(true);
  // Process Poweroff
  logic_state_machine_->process_event(arm_events::PowerOff());
  // Check arm is powered off
  ASSERT_FALSE(uav_arm_system_->enabled());
  // Process Poweron
  logic_state_machine_->process_event(arm_events::PowerOn());
  // Check arm is powered on
  ASSERT_TRUE(uav_arm_system_->enabled());
  // Check command status is false
  ASSERT_FALSE(arm_->getCommandStatus());
  // Check we can process fold
  logic_state_machine_->process_event(arm_events::Fold());
  ASSERT_TRUE(arm_->getCommandStatus());
  // Check we can process right fold
  logic_state_machine_->process_event(arm_events::RightAngleFold());
  ASSERT_TRUE(arm_->getCommandStatus());
  // Check we are still in Manual Control State
  ASSERT_STREQ(pstate(*logic_state_machine_), "ManualControlArmState");
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
