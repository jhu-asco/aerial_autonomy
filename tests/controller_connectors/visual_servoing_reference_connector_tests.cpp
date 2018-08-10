
#include <gtest/gtest.h>

#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_quad_connector.h>
#include <aerial_autonomy/controller_connectors/visual_servoing_reference_connector.h>
#include <aerial_autonomy/controllers/ddp_quad_mpc_controller.h>
#include <aerial_autonomy/controllers/quad_particle_reference_controller.h>
#include <aerial_autonomy/estimators/thrust_gain_estimator.h>
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

class VisualServoingReferenceConnectorTests : public ::testing::Test {
public:
  using VisualServoingReferenceConnectorT =
      VisualServoingReferenceConnector<Eigen::VectorXd, Eigen::VectorXd,
                                       MPCControllerQuadConnector>;

  VisualServoingReferenceConnectorTests()
      : tracking_offset_transform_(
            tf::createQuaternionFromRPY(0, M_PI / 3, M_PI / 2),
            tf::Vector3(0, 0, 0)),
        thrust_gain_estimator_(0.18), tol_(0.5) {
    // Drone settings
    drone_hardware_.usePerfectTime();
    drone_hardware_.set_delay_send_time(0.02);
    // Config
    mpc_config_ = createQuadMPCConfig();
    tf::Transform camera_transform = tf::Transform::getIdentity();
    simple_tracker_.reset(new SimpleTracker(drone_hardware_, camera_transform));
    auto duration = std::chrono::milliseconds(20);
    // Low-level:
    mpc_controller_.reset(new DDPQuadMPCController(mpc_config_, duration));
    quad_mpc_connector_.reset(new MPCControllerQuadConnector(
        drone_hardware_, *mpc_controller_, thrust_gain_estimator_));
    // High level
    reference_generator_.reset(
        new QuadParticleReferenceController(particle_reference_config_));
    visual_servoing_connector_.reset(new VisualServoingReferenceConnectorT(
        *simple_tracker_, drone_hardware_, *reference_generator_,
        *quad_mpc_connector_, camera_transform, tracking_offset_transform_));
  }

  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("visual_servoing_reference_connector");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("quad_mpc_state_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("ddp_quad_mpc_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("tracking_vector_estimator");
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
    visual_servoing_connector_->initialize();
    quad_mpc_connector_->initialize();
    // Run controller until inactive
    auto runController = [&]() {
      visual_servoing_connector_->run();
      quad_mpc_connector_->run();
      return quad_mpc_connector_->getStatus() == ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(
        runController, std::chrono::seconds(20), std::chrono::milliseconds(0)));
    // Check position is the goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    ASSERT_TF_NEAR(quad_transform,
                   gravity_aligned_tracked_pose * goal_relative_pose_tf, tol_);
    ASSERT_EQ(quad_mpc_connector_->getStatus(), ControllerStatus::Completed);
    ASSERT_NEAR(thrust_gain_estimator_.getThrustGain(), 0.16, 1e-2);
  }

  QuadSimulator drone_hardware_;
  std::unique_ptr<SimpleTracker> simple_tracker_;
  std::unique_ptr<QuadParticleReferenceController> reference_generator_;
  std::unique_ptr<VisualServoingReferenceConnectorT> visual_servoing_connector_;
  std::unique_ptr<DDPQuadMPCController> mpc_controller_;
  std::unique_ptr<MPCControllerQuadConnector> quad_mpc_connector_;
  tf::Transform tracking_offset_transform_;
  ThrustGainEstimator thrust_gain_estimator_;
  QuadMPCControllerConfig mpc_config_;
  ParticleReferenceConfig particle_reference_config_;
  double tol_;
};

TEST_F(VisualServoingReferenceConnectorTests, Constructor) {}

TEST_F(VisualServoingReferenceConnectorTests, InitializationTest) {
  // ASSERT_EQ(quad_mpc_connector_->getGoal(), nullptr);
  tf::Transform tracker_pose;
  tracker_pose.setIdentity();
  tracker_pose.setOrigin(tf::Vector3(1, 1, 1));
  simple_tracker_->setTargetPoseGlobalFrame(tracker_pose);
  simple_tracker_->setTrackingIsValid(true);
  visual_servoing_connector_->setGoal(PositionYaw());
  visual_servoing_connector_->initialize();
  auto reference = quad_mpc_connector_->getGoal();
  auto state_control = reference->atTime(20);
  Eigen::VectorXd goal = state_control.first;
  ASSERT_VEC_NEAR(tracker_pose.getOrigin(),
                  tf::Vector3(goal[0], goal[1], goal[2]), 1e-3);
  ASSERT_NEAR(goal[3], 0, 1e-2);
  ASSERT_NEAR(goal[4], 0, 1e-2);
  ASSERT_NEAR(goal[5], M_PI / 2.0, 1e-2);
}

TEST_F(VisualServoingReferenceConnectorTests, CriticalRun) {
  visual_servoing_connector_->setGoal(PositionYaw());
  // make tracking invalid:
  simple_tracker_->setTrackingIsValid(false);
  // Run connector
  visual_servoing_connector_->run();
  ASSERT_EQ(visual_servoing_connector_->getStatus(),
            ControllerStatus::Critical);
}

TEST_F(VisualServoingReferenceConnectorTests, RunUntilConvergence) {
  // set tracking goal
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, -0.1),
                             tf::Vector3(2, -0.5, 0.5));
  PositionYaw goal_relative_pose(1, 0, 0, 0.5);
  runUntilConvergence(tracked_pose, goal_relative_pose);
}

TEST_F(VisualServoingReferenceConnectorTests,
       RunUntilConvergenceNonZeroRollPitch) {
  // set tracking goal
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0.4, -0.6, -0.5),
                             tf::Vector3(1.5, -0.5, 0.5));
  PositionYaw goal_relative_pose(1, -1.5, 2, 0.5);
  runUntilConvergence(tracked_pose, goal_relative_pose);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
