#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/log/log.h>
#include <aerial_autonomy/state_machines/uav_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_system_handler.h>
#include <aerial_autonomy/uav_basic_events.h>

#include <glog/logging.h>

#include "log_config.pb.h"
#include "uav_system_handler_config.pb.h"

/**
 * @brief Loads configutation file and starts system
 * @param argc Number of arguments
 * @param argv Arguments
 * @return Exit status
 */
int main(int argc, char **argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InitGoogleLogging("aerial_autonomy_node");

  ros::init(argc, argv, "aerial_autonomy");
  ros::NodeHandle nh;

  std::string uav_system_config_filename;
  if (!nh.getParam("uav_system_config_filename", uav_system_config_filename)) {
    LOG(FATAL) << "ROS param \"uav_system_config_filename\" not found";
  }

  std::string log_config_filename;
  if (!nh.getParam("log_config_filename", log_config_filename)) {
    LOG(FATAL) << "ROS param \"log_config_filename\" not found";
  }

  UAVSystemHandlerConfig uav_system_config;
  if (!proto_utils::loadProtoText(uav_system_config_filename,
                                  uav_system_config)) {
    LOG(FATAL) << "Failed to open config file: " << uav_system_config_filename;
  }

  LogConfig log_config;
  if (!proto_utils::loadProtoText(log_config_filename, log_config)) {
    LOG(FATAL) << "Failed to open log config file: " << log_config_filename;
  }
  Log::instance().configure(log_config);

  UAVSystemHandler<UAVStateMachine,
                   uav_basic_events::UAVEventManager<UAVStateMachine>>
      uav_system_handler(uav_system_config);

  ros::spin();

  return 0;
}
