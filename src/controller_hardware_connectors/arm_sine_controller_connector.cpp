#include "aerial_autonomy/controller_hardware_connectors/arm_sine_controller_connector.h"

void ArmSineControllerConnector::sendHardwareCommands(
    std::vector<double> controls) {
  if (!arm_hardware_.setJointAngles(controls)) {
    LOG_EVERY_N(WARNING, 50) << "Failed to set joint angles";
  }
}
