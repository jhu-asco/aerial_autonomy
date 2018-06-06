#include <aerial_autonomy/common/mpc_trajectory_visualizer.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h>
#include <aerial_autonomy/controllers/ddp_airm_mpc_controller.h>
#include <aerial_autonomy/log/log.h>
#include <aerial_autonomy/types/spiral_reference_trajectory.h>
#include <aerial_autonomy/types/waypoint.h>

#include "arm_parsers/arm_simulator.h"
#include <quad_simulator_parser/quad_simulator.h>

#include <glog/logging.h>

/**
* @brief Create a way point reference trajectory from goal positionyaw and joint
* angles
*
* @param goal Goal position yaw
* @param desired_joint_angle_1 First joint angle
* @param desired_joint_angle_2 Second joint angle
*
* @return waypoint reference trajectory
*/
std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
createWayPoint(PositionYaw goal, double desired_joint_angle_1,
               double desired_joint_angle_2) {
  std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>> waypoint;
  Eigen::VectorXd goal_control(6);
  goal_control << 1, 0, 0, 0, desired_joint_angle_1, desired_joint_angle_2;
  Eigen::VectorXd goal_state(21);
  goal_state << goal.x, goal.y, goal.z, 0, 0, goal.yaw, 0, 0, 0, 0, 0, 0, 0, 0,
      goal.yaw, desired_joint_angle_1, desired_joint_angle_2, 0, 0,
      desired_joint_angle_1, desired_joint_angle_2;
  waypoint.reset(
      new Waypoint<Eigen::VectorXd, Eigen::VectorXd>(goal_state, goal_control));
  return waypoint;
}

/**
* @brief Create a spiral reference trajectory
*
* @param drone_hardware Quad simulator
*
* @return spiral reference trajectory
*/
std::shared_ptr<SpiralReferenceTrajectory>
createSpiralReference(quad_simulator::QuadSimulator &drone_hardware) {
  parsernode::common::quaddata data;
  drone_hardware.getquaddata(data);
  Eigen::Vector3d current_position(data.localpos.x, data.localpos.y,
                                   data.localpos.z);
  double current_yaw = data.rpydata.z;
  LOG(INFO) << "Current Position: " << current_position;
  LOG(INFO) << "Current yaw: " << current_yaw;
  SpiralReferenceTrajectoryConfig spiral_reference_config;
  if (!proto_utils::loadProtoText(std::string(PROJECT_SOURCE_DIR) +
                                      "/param/spiral_reference_config.pbtxt",
                                  spiral_reference_config)) {
    LOG(ERROR) << "Cannot load proto file for spiral reference trajectory";
  }
  ArmSineControllerConfig arm_sine_reference_config;
  if (!proto_utils::loadProtoText(std::string(PROJECT_SOURCE_DIR) +
                                      "/param/arm_sine_reference_config.pbtxt",
                                  arm_sine_reference_config)) {
    LOG(ERROR) << "Cannot load proto file for arm sine reference";
  }
  return std::shared_ptr<SpiralReferenceTrajectory>(
      new SpiralReferenceTrajectory(spiral_reference_config,
                                    arm_sine_reference_config, current_position,
                                    current_yaw, 0.16));
}

int main(int argc, char **argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InitGoogleLogging("mpc_control_tuning");
  ros::init(argc, argv, "mpc_control_tuning");
  ros::NodeHandle nh;
  LogConfig log_config;
  log_config.set_directory(std::string(PROJECT_SOURCE_DIR) + "/logs/data");
  auto data_stream_config = log_config.add_data_stream_configs();
  data_stream_config->set_log_rate(25);
  data_stream_config->set_stream_id("thrust_gain_estimator");
  Log::instance().configure(log_config);
  AirmMPCControllerConfig mpc_controller_config;
  if (!proto_utils::loadProtoText(std::string(PROJECT_SOURCE_DIR) +
                                      "/param/mpc_controller_config.pbtxt",
                                  mpc_controller_config)) {
    LOG(ERROR) << "Cannot load proto file for mpc controller";
  }
  quad_simulator::QuadSimulator drone_hardware;
  ArmSimulator arm_simulator;
  drone_hardware.usePerfectTime();
  drone_hardware.set_delay_send_time(0.02);
  ThrustGainEstimator thrust_gain_estimator_(0.16);
  DDPAirmMPCController controller(mpc_controller_config,
                                  std::chrono::milliseconds(20));
  MPCControllerAirmConnector controller_connector(
      drone_hardware, arm_simulator, controller, thrust_gain_estimator_);
  MPCVisualizerConfig visualizer_config;
  visualizer_config.mutable_trajectory_color()->set_g(1.0);
  visualizer_config.mutable_trajectory_color()->set_r(0.0);
  visualizer_config.mutable_desired_trajectory_color()->set_a(0.5);
  MPCTrajectoryVisualizer visualizer(controller_connector, visualizer_config);
  // auto reference_ptr = createWayPoint(PositionYaw(1,1,1, 1.0), -0.8, 1.4);
  auto reference_ptr = createSpiralReference(drone_hardware);
  controller_connector.usePerfectTimeDiff(
      0.02); ///\todo Remove this flag business
  // Start drone
  drone_hardware.setBatteryPercent(60);
  drone_hardware.takeoff();
  arm_simulator.setJointAngles(std::vector<double>{-0.7, 1.2});
  controller_connector.setGoal(reference_ptr);
  int count = 0;
  ROS_INFO("Please press any key to start...");
  getchar(); // Wait for user input
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
