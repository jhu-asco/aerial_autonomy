#include "aerial_autonomy/robot_systems/uav_vision_system.h"

#include <gtest/gtest.h>

TEST(UAVVisionSystemTests, Constructor) {
  ros::NodeHandle nh;
  UAVSystemConfig config;
  config.mutable_uav_vision_system_config()->set_tracker_type("ROI");
  auto ar_marker_direction_estimator_config =
      config.mutable_uav_vision_system_config()
          ->mutable_ar_marker_direction_estimator_config();
  for (int i = 0; i < 6; ++i) {
    ar_marker_direction_estimator_config->mutable_process_noise_stdev()->Add(
        1e-2);
    ar_marker_direction_estimator_config->mutable_meas_noise_stdev()->Add(1e-2);
    ar_marker_direction_estimator_config->mutable_init_state_stdev()->Add(1e-2);
  }
  config.set_uav_parser_type("quad_simulator_parser/QuadSimParser");
  ASSERT_NO_THROW(new UAVVisionSystem(config));
}

/// \todo Matt Add more tests!

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_vision_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
