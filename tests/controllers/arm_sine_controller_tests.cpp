#include "aerial_autonomy/controllers/arm_sine_controller.h"
#include "aerial_autonomy/log/log.h"
#include <chrono>
#include <gtest/gtest.h>

class ArmSineControllerTests : public ::testing::Test {
public:
  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("arm_sine_controller");
    Log::instance().addDataStream(data_config);
  }

  ArmSineControllerConfig createJointConfig(std::vector<double> amp,
                                            std::vector<double> freq,
                                            std::vector<double> phase) {
    ArmSineControllerConfig config;
    int N = amp.size();
    for (int i = 0; i < N; ++i) {
      auto p = config.mutable_joint_config()->Add();
      p->set_amplitude(amp.at(i));
      p->set_phase(phase.at(i));
      p->set_frequency(freq.at(i));
    }
    return config;
  }
  ArmSineControllerConfig createJointConfig() {
    std::vector<double> dummy;
    return createJointConfig(dummy, dummy, dummy);
  }

  std::pair<std::vector<double>, bool>
  runController(ArmSineController &controller) {
    std::vector<double> joint_angles;
    EmptySensor empty_sensor;
    bool status = controller.run(empty_sensor, joint_angles);
    return std::make_pair(joint_angles, status);
  }
};

TEST_F(ArmSineControllerTests, Constructor) {
  auto config = createJointConfig();
  ArmSineController controller(config);
  SUCCEED();
}

TEST_F(ArmSineControllerTests, CheckDuration) {
  auto config = createJointConfig();
  ArmSineController controller(config);
  controller.setZeroTime();
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  double duration = controller.duration().count();
  ASSERT_NEAR(duration, 1.0, 1e-2);
}

TEST_F(ArmSineControllerTests, CheckZeroFreq) {
  std::vector<double> amp = {1, 1};
  std::vector<double> freq = {0, 0};
  std::vector<double> phase = {M_PI / 2.0, 0};
  auto config = createJointConfig(amp, freq, phase);
  ArmSineController controller(config);
  std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  auto result = runController(controller);
  ASSERT_TRUE(result.second);
  auto joint_angles = result.first;
  ASSERT_EQ(joint_angles.size(), 2);
  ASSERT_DOUBLE_EQ(joint_angles.at(0), amp.at(0));
  ASSERT_DOUBLE_EQ(joint_angles.at(1), 0.0);
}

TEST_F(ArmSineControllerTests, CheckPeriod) {
  std::vector<double> amp = {1, 1};
  std::vector<double> freq = {1.0, 2.0};
  std::vector<double> phase = {M_PI / 2.0, 0};
  auto config = createJointConfig(amp, freq, phase);
  ArmSineController controller(config);
  controller.setZeroTime();
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  auto result1 = runController(controller);
  controller.setZeroTime();
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  auto result2 = runController(controller);
  // Check sizes
  ASSERT_TRUE(result1.second);
  ASSERT_TRUE(result2.second);
  // Check values
  auto joint_angles = result1.first;
  ASSERT_NEAR(joint_angles.at(1), 0.0, 1e-2);
  auto joint_angles2 = result2.first;
  ASSERT_NEAR(joint_angles2.at(0), amp.at(0), 1e-2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
