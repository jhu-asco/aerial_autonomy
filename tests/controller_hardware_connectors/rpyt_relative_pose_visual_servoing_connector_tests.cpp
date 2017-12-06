
#include <gtest/gtest.h>

#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/controller_hardware_connectors/rpyt_relative_pose_visual_servoing_connector.h>
#include <aerial_autonomy/controllers/rpyt_based_relative_pose_controller.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <chrono>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

using namespace test_utils;

class RPYTRelativePoseVisualConnectorTests : public ::testing::Test {
public:
  RPYTRelativePoseVisualConnectorTests()
      : goal_tolerance_position_(0.05), goal_tolerance_velocity_(0.1),
        goal_tolerance_yaw_(0.05), goal_tolerance_yaw_rate_(0.05),
        tracking_offset_transform_(
            tf::createQuaternionFromRPY(0, M_PI / 3, M_PI / 2),
            tf::Vector3(0, 0, 0)),
        thrust_gain_estimator_(0.18) {
    RPYTBasedRelativePoseControllerConfig config;
    auto velocity_relative_pose_config =
        config.mutable_velocity_based_relative_pose_controller_config();
    auto position_controller_config =
        velocity_relative_pose_config
            ->mutable_velocity_based_position_controller_config();
    position_controller_config->set_position_gain(1.0);
    position_controller_config->set_max_velocity(1.0);
    position_controller_config->set_yaw_gain(1.0);
    position_controller_config->set_max_yaw_rate(1.0);
    position_controller_config->set_yaw_i_gain(0.0);
    position_controller_config->set_position_i_gain(0.0);
    position_controller_config->set_position_saturation_value(0.0);
    position_controller_config->set_yaw_saturation_value(0.0);
    position_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(goal_tolerance_yaw_);
    auto position_tolerance =
        position_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    position_tolerance->set_x(goal_tolerance_position_);
    position_tolerance->set_y(goal_tolerance_position_);
    position_tolerance->set_z(goal_tolerance_position_);
    auto rpyt_velocity_controller_config =
        config.mutable_rpyt_based_velocity_controller_config();
    rpyt_velocity_controller_config->mutable_velocity_controller_config()
        ->set_goal_yaw_tolerance(goal_tolerance_yaw_);
    auto goal_velocity_tolerance =
        rpyt_velocity_controller_config->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    goal_velocity_tolerance->set_vx(goal_tolerance_velocity_);
    goal_velocity_tolerance->set_vy(goal_tolerance_velocity_);
    goal_velocity_tolerance->set_vz(goal_tolerance_velocity_);
    tf::Transform camera_transform = tf::Transform::getIdentity();
    simple_tracker_.reset(new SimpleTracker(drone_hardware_, camera_transform));
    controller_.reset(new RPYTBasedRelativePoseController(
        config, std::chrono::milliseconds(20)));
    ar_marker_direction_estimator_.reset(new ARMarkerDirectionEstimator(
        ARMarkerDirectionEstimatorConfig(), std::chrono::milliseconds(20)));
    visual_servoing_connector_.reset(
        new RPYTRelativePoseVisualServoingConnector(
            *simple_tracker_, drone_hardware_, *controller_,
            thrust_gain_estimator_, *ar_marker_direction_estimator_,
            camera_transform, tracking_offset_transform_));
    drone_hardware_.usePerfectTime();
  }

  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("rpyt_relative_pose_visual_servoing_connector");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("velocity_based_position_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("rpyt_based_velocity_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("velocity_based_relative_pose_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("ar_marker_direction_estimator");
    Log::instance().addDataStream(data_config);
  }

  void runUntilConvergence(const tf::Transform &tracked_pose,
                           const PositionYaw &goal_relative_pose) {
    simple_tracker_->setTargetPoseGlobalFrame(tracked_pose);
    simple_tracker_->setTrackingIsValid(true);
    tf::Transform gravity_aligned_tracked_pose =
        tracked_pose * tracking_offset_transform_;
    double roll, pitch, yaw;
    gravity_aligned_tracked_pose.getBasis().getRPY(roll, pitch, yaw);
    gravity_aligned_tracked_pose.getBasis().setRPY(0, 0, yaw);
    // Fly quadrotor which sets the altitude to 0.5
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    // Set goal
    tf::Transform goal_relative_pose_tf;
    conversions::positionYawToTf(goal_relative_pose, goal_relative_pose_tf);
    visual_servoing_connector_->setGoal(goal_relative_pose);
    // Run controller until inactive
    auto runController = [&]() {
      visual_servoing_connector_->run();
      return visual_servoing_connector_->getStatus() ==
             ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(
        runController, std::chrono::seconds(1), std::chrono::milliseconds(0)));
    // Check position is the goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    ASSERT_TF_NEAR(quad_transform,
                   gravity_aligned_tracked_pose * goal_relative_pose_tf,
                   goal_tolerance_yaw_);
    ASSERT_EQ(visual_servoing_connector_->getStatus(),
              ControllerStatus::Completed);
    ASSERT_NEAR(thrust_gain_estimator_.getThrustGain(), 0.16, 1e-4);
  }

  QuadSimulator drone_hardware_;
  std::unique_ptr<RPYTBasedRelativePoseController> controller_;
  std::unique_ptr<SimpleTracker> simple_tracker_;
  std::unique_ptr<RPYTRelativePoseVisualServoingConnector>
      visual_servoing_connector_;
  std::unique_ptr<ARMarkerDirectionEstimator> ar_marker_direction_estimator_;
  double goal_tolerance_position_;
  double goal_tolerance_velocity_;
  double goal_tolerance_yaw_;
  double goal_tolerance_yaw_rate_;
  tf::Transform tracking_offset_transform_;
  ThrustGainEstimator thrust_gain_estimator_;
};

TEST_F(RPYTRelativePoseVisualConnectorTests, Constructor) {}

TEST_F(RPYTRelativePoseVisualConnectorTests, CriticalRun) {
  // make tracking invalid:
  simple_tracker_->setTrackingIsValid(false);
  // Run connector
  visual_servoing_connector_->run();
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Critical);
}

TEST_F(RPYTRelativePoseVisualConnectorTests, RunUntilConvergence) {
  // set tracking goal
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, -0.1),
                             tf::Vector3(2, -0.5, 0.5));
  PositionYaw goal_relative_pose(1, 0, 0, 0.5);
  runUntilConvergence(tracked_pose, goal_relative_pose);
}

TEST_F(RPYTRelativePoseVisualConnectorTests,
       RunUntilConvergenceNonZeroRollPitch) {
  // set tracking goal
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0.4, -0.6, -0.5),
                             tf::Vector3(2, -0.5, 0.5));
  PositionYaw goal_relative_pose(1, -1.5, 2, 0.5);
  runUntilConvergence(tracked_pose, goal_relative_pose);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
