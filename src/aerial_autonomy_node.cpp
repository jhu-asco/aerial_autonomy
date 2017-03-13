#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/onboard_system_handler.h>
#include <aerial_autonomy/state_machines/basic_state_machine.h>

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <glog/logging.h>

#include "onboard_system_handler_config.pb.h"

using namespace google::protobuf; ///< Protobuf namespace

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

  std::string onboard_system_config_filename;
  if (!nh.getParam("onboard_system_config_filename",
                   onboard_system_config_filename)) {
    LOG(FATAL) << "ROS param \"onboard_system_config_filename\" not found";
  }

  OnboardSystemHandlerConfig onboard_system_config;
  int fd = open(onboard_system_config_filename.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(FATAL) << "Failed to open config file: "
               << onboard_system_config_filename;
  }
  io::FileInputStream fstream(fd);
  if (!TextFormat::Parse(&fstream, &onboard_system_config)) {
    LOG(FATAL) << "Failed to parse config file: "
               << onboard_system_config_filename;
  }

  OnboardSystemHandler<LogicStateMachine,
                       basic_events::BasicEventManager<LogicStateMachine>>
      onboard_system_handler(nh, onboard_system_config);

  ros::spin();

  return 0;
}
