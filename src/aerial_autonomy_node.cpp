#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/onboard_system_handler.h>
#include <aerial_autonomy/state_machines/basic_state_machine.h>

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "onboard_system_handler_config.pb.h"

using namespace google::protobuf;

int main(int argc, char **argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::init(argc, argv, "aerial_autonomy");
  ros::NodeHandle nh;

  std::string onboard_system_config_filename;
  if (!nh.getParam("onboard_system_config_filename",
                   onboard_system_config_filename)) {
    std::cout << "ROS param \"onboard_system_config_filename\" not found"
              << std::endl;
    return -1;
  }

  OnboardSystemHandlerConfig onboard_system_config;
  int fd = open(onboard_system_config_filename.c_str(), O_RDONLY);
  if (fd < 0) {
    std::cout << "Failed to open config file: "
              << onboard_system_config_filename << std::endl;
    return -1;
  }
  io::FileInputStream fstream(fd);
  if (!TextFormat::Parse(&fstream, &onboard_system_config)) {
    std::cout << "Failed to parse config file: "
              << onboard_system_config_filename << std::endl;
    return -1;
  }

  OnboardSystemHandler<LogicStateMachine,
                       basic_events::BasicEventManager<LogicStateMachine>>
      onboard_system_handler(nh, onboard_system_config);

  ros::spin();
}
