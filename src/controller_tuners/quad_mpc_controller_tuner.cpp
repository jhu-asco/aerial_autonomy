#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/mpc_trajectory_visualizer.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_quad_connector.h>
#include <aerial_autonomy/controllers/ddp_quad_mpc_controller.h>
#include <aerial_autonomy/log/log.h>
#include <aerial_autonomy/types/quad_particle_reference_trajectory.h>
#include <aerial_autonomy/types/waypoint.h>

#include <quad_simulator_parser/quad_simulator.h>

#include <glog/logging.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

/**
* @brief Create a Quad Particle trajectory
*
* @param drone_hardware Quad simulator
* @param delta change in position yaw
*
*/
std::shared_ptr<QuadParticleTrajectory>
createQuadParticleReference(quad_simulator::QuadSimulator &drone_hardware,
                            PositionYaw goal_position_yaw) {
  parsernode::common::quaddata data;
  drone_hardware.getquaddata(data);
  PositionYaw start_position_yaw(data.localpos.x, data.localpos.y,
                                 data.localpos.z, data.rpydata.z);
  ROS_INFO("Current Pos: %f, %f, %f, %f", start_position_yaw.x,
           start_position_yaw.y, start_position_yaw.z, start_position_yaw.yaw);
  ParticleReferenceConfig particle_reference_config;
  if (!proto_utils::loadProtoText(std::string(PROJECT_SOURCE_DIR) +
                                      "/param/particle_reference_config.pbtxt",
                                  particle_reference_config)) {
    LOG(WARNING) << "Cannot load proto file for particle reference trajectory";
  }
  return std::shared_ptr<QuadParticleTrajectory>(new QuadParticleTrajectory(
      goal_position_yaw, start_position_yaw, particle_reference_config));
}

void setPose(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped,
             MPCControllerQuadConnector &connector,
             quad_simulator::QuadSimulator &drone_hardware) {
  auto pose = pose_stamped->pose;
  PositionYaw goal_position_yaw(pose.position.x, pose.position.y,
                                pose.position.z, tf::getYaw(pose.orientation));
  ROS_INFO("goal: %f, %f, %f, %f", goal_position_yaw.x, goal_position_yaw.y,
           goal_position_yaw.z, goal_position_yaw.yaw);
  auto reference_ptr =
      createQuadParticleReference(drone_hardware, goal_position_yaw);
  connector.setGoal(reference_ptr);
}

int main(int argc, char **argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InitGoogleLogging("quad_mpc_control_tuning");
  ros::init(argc, argv, "quad_mpc_control_tuning");
  ros::NodeHandle nh;
  LogConfig log_config;
  log_config.set_directory(std::string(PROJECT_SOURCE_DIR) + "/logs/data");
  auto data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(25);
  data_stream_config->set_stream_id("thrust_gain_estimator");
  data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(10);
  data_stream_config->set_stream_id("quad_mpc_state_estimator");
  data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(10);
  data_stream_config->set_stream_id("ddp_quad_mpc_controller");
  Log::instance().configure(log_config);
  QuadMPCControllerConfig quad_mpc_controller_config;
  if (!proto_utils::loadProtoText(std::string(PROJECT_SOURCE_DIR) +
                                      "/param/quad_mpc_controller_config.pbtxt",
                                  quad_mpc_controller_config)) {
    LOG(ERROR) << "Cannot load proto file for mpc controller";
  }
  quad_simulator::QuadSimulator drone_hardware;
  drone_hardware.usePerfectTime();
  drone_hardware.set_delay_send_time(0.02);
  // drone_hardware.set_delay_send_time(0.2);
  ThrustGainEstimator thrust_gain_estimator_(0.16);
  DDPQuadMPCController controller(quad_mpc_controller_config,
                                  std::chrono::milliseconds(20));
  MPCControllerQuadConnector controller_connector(drone_hardware, controller,
                                                  thrust_gain_estimator_);
  MPCVisualizerConfig visualizer_config;
  visualizer_config.set_skip_segments(10);
  visualizer_config.mutable_trajectory_color()->set_g(1.0);
  visualizer_config.mutable_trajectory_color()->set_r(0.0);
  visualizer_config.mutable_desired_trajectory_color()->set_a(0.5);
  MPCTrajectoryVisualizer visualizer(controller_connector, visualizer_config);
  PositionYaw goal_position_yaw(1.0, 1.0, 1.0, 0.5);
  // auto reference_ptr =
  //    conversions::createQuadWayPoint(goal_position_yaw);
  auto reference_ptr =
      createQuadParticleReference(drone_hardware, goal_position_yaw);
  controller_connector.usePerfectTimeDiff(
      0.02); ///\todo Remove this flag business
  // Start drone
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
  controller_connector.initialize();
  controller_connector.setGoal(reference_ptr);
  controller_connector.initialize();
  // Setting up subscriber
  ros::Subscriber ref_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1,
      boost::bind(setPose, _1, boost::ref(controller_connector),
                  boost::ref(drone_hardware)));
  int count = 0;
  while (ros::ok()) {
    controller_connector.run();
    if (++count == 4) {
      visualizer.publishTrajectory();
      count = 0;
    }
    LOG_EVERY_N(INFO, 20) << "Connector: "
                          << controller_connector.getStatus().statusAsText();
    ros::spinOnce();
  }
  return 0;
}
