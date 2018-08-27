#include "aerial_autonomy/types/minimum_snap_reference_trajectory.h"
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/common/qrotor_backstepping_trajectory_visualizer.h>
#include <aerial_autonomy/log/log.h>
#include <chrono>
#include <glog/logging.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InitGoogleLogging("qrotor_backstepping_control_tuning");
  ros::init(argc, argv, "qrotor_backstepping_control_tuning");
  ros::NodeHandle nh;
  LogConfig log_config;
  log_config.set_directory(std::string(PROJECT_SOURCE_DIR) + "/logs/data");
  auto data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(25);
  data_stream_config->set_stream_id("thrust_gain_estimator");
  data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(10);
  data_stream_config->set_stream_id("qrotor_backstepping_controller");
  data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(10);
  data_stream_config->set_stream_id("qrotor_backstepping_controller_connector");
  Log::instance().configure(log_config);
  QrotorBacksteppingControllerConfig config;
  if (!proto_utils::loadProtoText(
          std::string(PROJECT_SOURCE_DIR) +
              "/param/qrotor_backstepping_controller_config.pbtxt",
          config)) {
    LOG(ERROR) << "Cannot load proto file for the controller";
  }
  quad_simulator::QuadSimulator drone_hardware;
  drone_hardware.usePerfectTime();
  drone_hardware.set_delay_send_time(0.2);
  ThrustGainEstimator thrust_gain_estimator(0.16);
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();

  QrotorBacksteppingController controller(config);
  QrotorBacksteppingControllerConnector controller_connector(
      drone_hardware, controller, thrust_gain_estimator, config);
  MPCVisualizerConfig visualizer_config;
  visualizer_config.mutable_desired_trajectory_color()->set_r(1);
  visualizer_config.mutable_desired_trajectory_color()->set_g(0);
  visualizer_config.mutable_desired_trajectory_color()->set_b(0);
  visualizer_config.mutable_desired_trajectory_color()->set_a(0.8);
  QrotorBacksteppingTrajectoryVisualizer visualizer(visualizer_config);

  // Set goal
  int r = 4;
  Eigen::VectorXd tau_vec(4);
  tau_vec << 11.5, 11.5, 11.5, 11.5;
  Eigen::MatrixXd path(5, 3);
  path << 0, 0, 0, 10, 0, 0, 10, 10, 3, 0, 10, 0, 0, 0, 0;
  std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal(
      new MinimumSnapReferenceTrajectory(r, tau_vec, path));
  controller_connector.setGoal(goal);

  // Initial condition
  ParticleState initial_desired_state = std::get<0>(goal->atTime(0.0));
  geometry_msgs::Vector3 init_position;
  init_position.x = initial_desired_state.p.x;
  init_position.y = initial_desired_state.p.y - .3;
  init_position.z = initial_desired_state.p.z - 8.0;
  drone_hardware.cmdwaypoint(init_position);

  parsernode::common::quaddata data;
  tf::TransformBroadcaster broadcaster;

  bool completed = false;

  // Start time
  auto t0 = std::chrono::high_resolution_clock::now();

  while (ros::ok()) {
    // Publish reference trajectory
    visualizer.publishTrajectory(goal, tau_vec, t0);

    // Vizualize quad frame
    drone_hardware.getquaddata(data);
    tf::Quaternion q = tf::createQuaternionFromRPY(
        data.rpydata.x, data.rpydata.y, data.rpydata.z);
    broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(
            q, tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z)),
        ros::Time::now(), "optitrak", "quad"));

    // Run controller
    controller_connector.run();
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Display controller status
    LOG_EVERY_N(INFO, 100) << "Connector: "
                           << controller_connector.getStatus().statusAsText();

    // Display completion time
    if (controller_connector.getStatus() == ControllerStatus::Completed &&
        !completed) {
      auto t1 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<float> delta_t = t1 - t0;
      std::cout << " Actual time vs assigned time to reach end: "
                << delta_t.count() << " seconds vs " << tau_vec.sum()
                << " seconds\n";
      completed = true;
    }
  }
  return 0;
}
