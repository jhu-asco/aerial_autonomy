﻿#include <aerial_autonomy/common/system_handler_node_utils.h>
#include <aerial_autonomy/mpc_events.h>
#include <aerial_autonomy/state_machines/mpc_state_machine.h>
#include <aerial_autonomy/system_handlers/mpc_system_handler.h>

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

  MPCSystemHandler<MPCStateMachine,
                   mpc_events::MPCEventManager<MPCStateMachine>>
      mpc_handler(uav_system_handler_config, state_machine_config);

  ros::spin();

  return 0;
}
