#include <aerial_autonomy/common/system_handler_node_utils.h>
#include <aerial_autonomy/state_machines/uav_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_system_handler.h>
#include <aerial_autonomy/uav_basic_events.h>

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
  BaseStateMachineConfig state_machine_config = createStateMachineConfig(nh);
  UAVSystemHandlerConfig uav_system_handler_config =
      createUAVSystemHandlerConfig(nh);

  UAVSystemHandler<UAVStateMachine,
                   uav_basic_events::UAVEventManager<UAVStateMachine>>
      uav_system_handler(uav_system_handler_config, state_machine_config);

  ros::spin();

  return 0;
}
