#include <aerial_autonomy/state_machines/orange_picking_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <arm_parsers/arm_simulator.h>
#include <gtest/gtest.h>
// Timer stuff
#include <chrono>
#include <thread>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief Namespaces for events such as GoHome and Pick
*/
namespace vse = visual_servoing_events;
namespace ope = orange_picking_events;

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

class OrangePickingStateMachineTests : public ::testing::Test {
public:
  OrangePickingStateMachineTests()
      : drone_hardware_(new QuadSimulator), arm_(new ArmSimulator),
        goal_tolerance_position_(1.0), grip_timeout_(25000),
        grip_duration_(10) {
    auto vision_state_machine_config =
        state_machine_config_.mutable_visual_servoing_state_machine_config();
    auto sensor_place_state_machine_config =
        vision_state_machine_config
            ->mutable_sensor_place_state_machine_config();
    // Set visual servoing controller
    vision_state_machine_config->set_connector_type(
        VisualServoingStateMachineConfig::RPYTPose);
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
    sensor_place_state_machine_config->set_grip_timeout(grip_timeout_);

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

    // Arm goal transforms
    // Place Arm Transform
    setTransform(sensor_place_state_machine_config->add_arm_goal_transform(),
                 -0.3, 0, 0, 0, 0, 0);
    // Checking Arm Transform
    setTransform(sensor_place_state_machine_config->add_arm_goal_transform(),
                 -0.5, 0, 0, 0, 0, 0);

    // Set Thresholds for Normal Forces
    sensor_place_state_machine_config->set_placing_acc_threshold(-1);
    sensor_place_state_machine_config->set_checking_acc_threshold(1);

    // Set Acceleration Bias Config
    auto acceleration_estimator_config =
        uav_vision_system_config->mutable_acceleration_bias_estimator_config();
    acceleration_estimator_config->set_max_bias(10);

    // Relative marker goal for pre-place
    auto pose_goal = vision_state_machine_config->add_relative_pose_goals();
    auto pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(0.75);
    pose_goal_position->set_y(0);
    pose_goal_position->set_z(0);
    pose_goal->set_yaw(0);
    // Relative marker goal for place
    pose_goal = vision_state_machine_config->add_relative_pose_goals();
    pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(0.25);
    pose_goal_position->set_y(0);
    pose_goal_position->set_z(0);
    pose_goal->set_yaw(0);
    // Relative marker goal for checking
    pose_goal = vision_state_machine_config->add_relative_pose_goals();
    pose_goal_position = pose_goal->mutable_position();
    pose_goal_position->set_x(0.5);
    pose_goal_position->set_y(0);
    pose_goal_position->set_z(0);
    pose_goal->set_yaw(0);

    // Acceleration Threshold
    sensor_place_state_machine_config->set_placing_acc_threshold(-5);
    sensor_place_state_machine_config->set_checking_acc_threshold(5);

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
    // Fill Path Sensor Config
    config_.set_use_path_sensor(true);
    auto path_sensor_config = config_.mutable_rpyt_reference_connector_config();
    path_sensor_config->set_final_time(3);
    path_sensor_config->mutable_ros_sensor_config()->set_topic("/path_topic");
    path_sensor_config->mutable_ros_sensor_config()->set_timeout(4);
    path_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/path_topic",1);

    tf::Transform camera_transform = conversions::protoTransformToTf(
        uav_vision_system_config->camera_transform());
    tracker_.reset(new SimpleTracker(*drone_hardware_, camera_transform));
    uav_arm_system_.reset(new UAVArmSystem(
        config_, std::dynamic_pointer_cast<BaseTracker>(tracker_),
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware_),
        std::dynamic_pointer_cast<ArmSimulator>(arm_)));
    logic_state_machine_.reset(new OrangePickingStateMachine(
        boost::ref(*uav_arm_system_), boost::cref(state_machine_config_)));
    logic_state_machine_->start();
    // Move to landed state
    logic_state_machine_->process_event(InternalTransitionEvent());

    // Target for place
    // Target objects
    targets_[0] =
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 0, 0.5));
    tracker_->setTargetPosesGlobalFrame(targets_);
    // Use simulated time for drone hardware
    drone_hardware_->usePerfectTime();
    arm_->grip(true);
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
    data_config.set_stream_id("quad_mpc_state_estimator");
    Log::instance().addDataStream(data_config);
  }

  ~OrangePickingStateMachineTests() {
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
  std::unique_ptr<OrangePickingStateMachine> logic_state_machine_;
  double goal_tolerance_position_;
  uint32_t grip_timeout_;
  uint32_t grip_duration_;
  std::unordered_map<uint32_t, tf::Transform> targets_;

  template <class EventT> void testManualControlAbort() {
    // Process Event
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
    // Process Event
    logic_state_machine_->process_event(EventT());
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STRNE(pstate(*logic_state_machine_), "Hovering");
    // Power off arm
    arm_->sendCmd(ArmParser::Command::POWER_OFF);
    // Check we are in Hovering
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
    // Power on arm
    arm_->sendCmd(ArmParser::Command::POWER_ON);
  }

  void GoToHoverFromLanded() {
    drone_hardware_->setBatteryPercent(100);
    logic_state_machine_->process_event(be::Takeoff());
    // Powers on arm and folds it; Starts takeoff
    logic_state_machine_->process_event(InternalTransitionEvent());
    // Completes takeoff and goes to hovering
    logic_state_machine_->process_event(InternalTransitionEvent());
  }

  void GoToPickFromHover() {
    logic_state_machine_->process_event(ope::Pick());
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  }

  void sendSampleMessage() {
    trajectory_msgs::JointTrajectory path_msg;
    //Build path_msg
    path_msg.header.stamp = ros::Time::now();
    for (int ii = 0; ii < 3; ++ii) {
      trajectory_msgs::JointTrajectoryPoint point;
      //xyz
      point.positions[0] = 0.1*ii;
      point.positions[1] = 0.2*ii;
      point.positions[2] = 0.3*ii;
      //rotlog
      point.positions[3] = 0.0;
      point.positions[4] = 0.0;
      point.positions[5] = 0.0;
      //velocity xyz
      point.velocities[0] = 0.1;
      point.velocities[1] = 0.2;
      point.velocities[2] = 0.3;
      //ang vel
      point.velocities[3] = 0.0;
      point.velocities[4] = 0.0;
      point.velocities[5] = 0.0;
      //accelerations
      point.accelerations[0] = 0.0;
      point.accelerations[1] = 0.0;
      point.accelerations[2] = 0.0;
      path_msg.points.push_back(point);
    }
    //velocity xyz
    path_msg.points[2].velocities[0] = 0.0;
    path_msg.points[2].velocities[1] = 0.0;
    path_msg.points[2].velocities[2] = 0.0;
    //Send path_msg
    path_pub.publish(odom_msg);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  void SimulatorPickFromHover() {
    GoToPickFromHover();
    // Check UAV and arm controllers are active//TODO
    ASSERT_EQ(
        uav_arm_system_->getStatus<OrangePickingReferenceConnector>(),
        ControllerStatus::Active);
    // Keep running the controller until its completed or timeout
    sendSampleMessage();
    auto getStatusRunControllers = [&]() {
      uav_arm_system_->runActiveController(ControllerGroup::UAV);
      uav_arm_system_->runActiveController(ControllerGroup::Arm);
      return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
                 ControllerStatus::Active ||
             uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
                 ControllerStatus::Active;
    };
    ASSERT_TRUE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                              std::chrono::seconds(2.5),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "PickState");
  }
  
  ros::NodeHandle nh;
  ros::Publisher path_pub;

};

TEST_F(OrangePickingStateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine_), "Landed");
  GoToHoverFromLanded();
}

/// \brief Test Arm folding during landing and takeoff
TEST_F(OrangePickingStateMachineTests, HoveringandLanding) {
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

/// \brief Test Pick Place
// Try full picking task
TEST_F(OrangePickingStateMachineTests, OrangePicking) {

  GoToHoverFromLanded();
  SimulatorPickFromHover();
  logic_state_machine_->process_event(Completed())
  ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
}

/// \brief Test Pick Time Out
TEST_F(OrangePickingStateMachineTests, PickTimeout) {
    GoToHoverFromLanded();
    GoToPickFromHover();
    // Check UAV and arm controllers are active//TODO
    ASSERT_EQ(
        uav_arm_system_->getStatus<OrangePickingReferenceConnector>(),
        ControllerStatus::Active);
    // Keep running the controller until its completed or timeout
    sendSampleMessage();
    auto getStatusRunControllers = [&]() {
      uav_arm_system_->runActiveController(ControllerGroup::UAV);
      uav_arm_system_->runActiveController(ControllerGroup::Arm);
      return uav_arm_system_->getActiveControllerStatus(ControllerGroup::UAV) ==
                 ControllerStatus::Active ||
             uav_arm_system_->getActiveControllerStatus(ControllerGroup::Arm) ==
                 ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(getStatusRunControllers,
                                              std::chrono::seconds(5),
                                              std::chrono::milliseconds(0)));
    logic_state_machine_->process_event(InternalTransitionEvent());
    ASSERT_STREQ(pstate(*logic_state_machine_), "Hovering");
}

// Manual rc abort
TEST_F(OrangePickingStateMachineTests, ManualControlAbort) {
  // First Takeoff
  GoToHoverFromLanded();
  // Pick
  testManualControlAbort<ope::Pick>();
}
// Arm abort
TEST_F(OrangePickingStateMachineTests, ArmOffAbort) {
  // First Takeoff
  GoToHoverFromLanded();
  // Pick
  testArmOffAbort<ope::Pick>();
}

// Manual control internal actions
TEST_F(OrangePickingStateMachineTests, OrangePickingManualControlInternalActions) {
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
  // Check command status is true
  ASSERT_TRUE(arm_->getGripperValue());
  // Check we can process fold
  logic_state_machine_->process_event(arm_events::Fold());
  ASSERT_TRUE(arm_->getGripperValue());
  // Check we can process right fold
  logic_state_machine_->process_event(arm_events::RightAngleFold());
  ASSERT_TRUE(arm_->getGripperValue());
  // Check we are still in Manual Control State
  ASSERT_STREQ(pstate(*logic_state_machine_), "ManualControlArmState");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
