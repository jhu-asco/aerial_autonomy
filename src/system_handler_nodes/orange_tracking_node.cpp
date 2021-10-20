#include <aerial_autonomy/common/system_handler_node_utils.h>
#include <aerial_autonomy/orange_tracking_events.h>
#include <aerial_autonomy/state_machines/orange_tracking_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_arm_system_handler.h>

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

  createAndConfigureLogConfig(nh);
  auto state_machine_config = loadConfigFromROSParam<BaseStateMachineConfig>(
      nh, "state_machine_config_filename");
  auto uav_system_handler_config =
      loadConfigFromROSParam<UAVSystemHandlerConfig>(
          nh, "uav_system_config_filename");

  UAVArmSystemHandler<
      OrangeTrackingStateMachine,
      orange_tracking_events::OrangeTrackingEventManager<OrangeTrackingStateMachine>>
      uav_system_handler(uav_system_handler_config, state_machine_config);

  ros::spin();

  return 0;
}
