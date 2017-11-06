#include "uav_system_handler_config.pb.h"
#include <aerial_autonomy/common/system_handler_node_utils.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(SystemHandlerNodeUtilsTests, configureLog) {
  ros::NodeHandle nh("~");
  createAndConfigureLogConfig(nh);
  // Check log is configured correctly
  ASSERT_TRUE(
      boost::filesystem::equivalent(Log::instance().directory().parent_path(),
                                    boost::filesystem::path("/tmp")));
  DataStream &stream = Log::instance()["velocity_based_position_controller"];
  auto config = stream.configuration();
  ASSERT_EQ(config.log_rate(), 20);
  DataStream &stream2 = Log::instance()["builtin_position_controller"];
  auto config2 = stream2.configuration();
  ASSERT_EQ(config2.log_rate(), 30);
}

TEST(SystemHandlerNodeUtilsTests, loadProtoConfigFromROSParam) {
  ros::NodeHandle nh("~");
  auto uav_system_handler_config =
      loadConfigFromROSParam<UAVSystemHandlerConfig>(
          nh, "uav_system_handler_config_filename");
  ASSERT_STREQ(
      uav_system_handler_config.uav_system_config().uav_parser_type().c_str(),
      "dji_parser/DjiParser");
}

TEST(SystemHandlerNodeUtilsTests, loadProtoNonExistentParamAndFile) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ros::NodeHandle nh("~");
  nh.setParam("non_existent_file", "");
  ASSERT_DEATH(
      {
        loadConfigFromROSParam<UAVSystemHandlerConfig>(nh, "non_existent_file");
      },
      "Failed to open config file: ");
  ASSERT_DEATH(
      {
        loadConfigFromROSParam<UAVSystemHandlerConfig>(nh,
                                                       "non_existent_param");
      },
      "ROS param \"non_existent_param\" not found");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "system_handler_utils_tests");
  return RUN_ALL_TESTS();
}
