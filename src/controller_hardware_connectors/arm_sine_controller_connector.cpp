#include "aerial_autonomy/controller_hardware_connectors/arm_sine_controller_connector.h"
#include "aerial_autonomy/log/log.h"

ArmSineControllerConnector::ArmSineControllerConnector(
    ArmParser &arm_hardware, ArmSineController &controller)
    : ControllerHardwareConnector(controller, HardwareType::Arm),
      arm_hardware_(arm_hardware), private_ref_controller_(controller) {
  // \todo Add a function to get number of joints instead of calling joint
  // angles to arm parser
  std::vector<double> joint_angles = arm_hardware_.getJointAngles();
  Log::instance()["arm_sine_controller_connector"] << DataStream::starth;
  int N = joint_angles.size();
  for (int i = 0; i < N; ++i) {
    std::string ja_header = "Ja_" + std::to_string(i);
    std::string jv_header = "Jv_" + std::to_string(i);
    Log::instance()["arm_sine_controller_connector"] << ja_header << jv_header;
  }
  Log::instance()["arm_sine_controller_connector"] << DataStream::endl;
}

void ArmSineControllerConnector::sendHardwareCommands(
    std::vector<double> controls) {
  if (!arm_hardware_.setJointAngles(controls)) {
    LOG_EVERY_N(WARNING, 50) << "Failed to set joint angles";
  }
}

bool ArmSineControllerConnector::extractSensorData(EmptySensor &) {
  std::vector<double> joint_angles = arm_hardware_.getJointAngles();
  std::vector<double> joint_velocities = arm_hardware_.getJointVelocities();
  Log::instance()["arm_sine_controller_connector"] << DataStream::startl;
  int N = joint_angles.size();
  for (int i = 0; i < N; ++i) {
    Log::instance()["arm_sine_controller_connector"] << joint_angles.at(i)
                                                     << joint_velocities.at(i);
  }
  Log::instance()["arm_sine_controller_connector"] << DataStream::endl;
  return true;
}
