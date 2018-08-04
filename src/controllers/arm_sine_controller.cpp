#include "aerial_autonomy/controllers/arm_sine_controller.h"
#include "aerial_autonomy/log/log.h"
#include <string>

ArmSineController::ArmSineController(ArmSineControllerConfig config)
    : config_(config) {
  Log::instance()["arm_sine_controller"] << DataStream::starth;
  for (int i = 0; i < config_.joint_config_size(); ++i) {
    std::string header = "Jad_" + std::to_string(i);
    Log::instance()["arm_sine_controller"] << header;
  }
  Log::instance()["arm_sine_controller"] << DataStream::endl;
}

void ArmSineController::setZeroTime() {
  t0_ = std::chrono::high_resolution_clock::now();
}

std::chrono::duration<double> ArmSineController::duration() {
  return std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now() - t0_);
}

bool ArmSineController::runImplementation(EmptySensor, EmptyGoal,
                                          JointAngles &control) {
  auto joint_config = config_.joint_config();
  Log::instance()["arm_sine_controller"] << DataStream::startl;
  for (auto it = joint_config.begin(); it < joint_config.end(); ++it) {
    double a = it->amplitude();
    double omega = 2 * M_PI * it->frequency();
    double phi = it->phase();
    double dt = duration().count();
    double angle = phi + a * sin(omega * dt);
    control.push_back(angle);
    Log::instance()["arm_sine_controller"] << angle;
  }
  Log::instance()["arm_sine_controller"] << DataStream::endl;
  return true;
}
