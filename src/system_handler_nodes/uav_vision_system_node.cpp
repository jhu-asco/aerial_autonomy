#include <aerial_autonomy/state_machines/visual_servoing_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_vision_system_handler.h>
#include <aerial_autonomy/uav_basic_events.h>

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <glog/logging.h>

#include "uav_system_handler_config.pb.h"

/**
* @brief Protobuf namespace
*/
using namespace google::protobuf;

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

  UAVSystemHandlerConfig uav_system_config;
  int fd = open(uav_system_config_filename.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(FATAL) << "Failed to open config file: " << uav_system_config_filename;
  }
  io::FileInputStream fstream(fd);
  if (!TextFormat::Parse(&fstream, &uav_system_config)) {
    LOG(FATAL) << "Failed to parse config file: " << uav_system_config_filename;
  }

  UAVVisionSystemHandler<VisualServoingStateMachine,
                         visual_servoing_events::VisualServoingEventManager<
                             VisualServoingStateMachine>>
      uav_system_handler(nh, uav_system_config);

  ros::spin();

  return 0;
}
