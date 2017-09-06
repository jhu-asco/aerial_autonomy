#include <aerial_autonomy/common/system_handler_node_utils.h>
#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/state_machines/pick_place_state_machine.h>
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
  BaseStateMachineConfig state_machine_config = createStateMachineConfig(nh);
  UAVSystemHandlerConfig uav_system_handler_config =
      createUAVSystemHandlerConfig(nh);

  UAVArmSystemHandler<
      PickPlaceStateMachine,
      pick_place_events::PickPlaceEventManager<PickPlaceStateMachine>>
      uav_system_handler(uav_system_handler_config, state_machine_config);

  ros::spin();

  return 0;
}
