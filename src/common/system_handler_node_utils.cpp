#include <aerial_autonomy/common/system_handler_node_utils.h>

void createAndConfigureLogConfig(ros::NodeHandle &nh) {
  LOG(INFO) << "System Handler Node Utils func";//TAGGED
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
