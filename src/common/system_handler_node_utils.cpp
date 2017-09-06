#include <aerial_autonomy/common/system_handler_node_utils.h>

void createAndConfigureLogConfig(ros::NodeHandle &nh) {
  std::string log_config_filename;
  if (!nh.getParam("log_config_filename", log_config_filename)) {
    LOG(FATAL) << "ROS param \"log_config_filename\" not found";
  }

  LogConfig log_config;
  if (!proto_utils::loadProtoText(log_config_filename, log_config)) {
    LOG(FATAL) << "Failed to open log config file: " << log_config_filename;
  }
  Log::instance().configure(log_config);
}

BaseStateMachineConfig createStateMachineConfig(ros::NodeHandle &nh) {
  std::string state_machine_config_filename;
  if (!nh.getParam("state_machine_config_filename",
                   state_machine_config_filename)) {
    LOG(FATAL) << "ROS param \"state_machine_config_filename\" not found";
  }

  BaseStateMachineConfig state_machine_config;
  if (!proto_utils::loadProtoText(state_machine_config_filename,
                                  state_machine_config)) {
    LOG(FATAL) << "Failed to open state machine config file: "
               << state_machine_config_filename;
  }
  return state_machine_config;
}

UAVSystemHandlerConfig createUAVSystemHandlerConfig(ros::NodeHandle &nh) {
  std::string uav_system_config_filename;
  if (!nh.getParam("uav_system_config_filename", uav_system_config_filename)) {
    LOG(FATAL) << "ROS param \"uav_system_config_filename\" not found";
  }

  UAVSystemHandlerConfig uav_system_config;
  if (!proto_utils::loadProtoText(uav_system_config_filename,
                                  uav_system_config)) {
    LOG(FATAL) << "Failed to open config file: " << uav_system_config_filename;
  }
  return uav_system_config;
}
