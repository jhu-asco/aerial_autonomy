#include <aerial_autonomy/common/configuration_interface.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct Config1 {
  int i = 1;
};
struct Config2 {
  int j = 2;
};
struct TargetState1 {};
struct TargetState2 {};
struct TargetState3 {};
///

/// \brief TEST
/// All the tests are defined here
TEST(WrapperTests, SaveAndRetrieveObject) {
  LOG(INFO) << "Hello";
  Config1 original;
  Config1 retrieved;
  original.i = 5;
  ProtoBaseWrapper<Config1> config1_wrapper(original);
  // Retrieve object
  retrieved = config1_wrapper.getInput();
  EXPECT_EQ(retrieved.i, original.i);
  // Modifying
  Config1 &retrieved_ref = config1_wrapper.getInput();
  retrieved_ref.i = 10;
  retrieved = config1_wrapper.getInput();
  EXPECT_EQ(retrieved.i, retrieved_ref.i);
}

TEST(ConfigurationInterfaceTests, SaveAndRetrieveConfigs) {
  ConfigurationInterface configuration_interface;
  Config1 config1;
  config1.i = 10;
  Config2 config2;
  // Add configs to interface
  configuration_interface.addConfig<TargetState1, Config1>(config1);
  configuration_interface.addConfig<TargetState2, Config2>(config2);
  // Retrieve configs
  auto &retrieved_config1 =
      configuration_interface.getConfig<Config1, TargetState1>();
  EXPECT_EQ(retrieved_config1.i, config1.i);
  // Try changing the retrieved one
  retrieved_config1.i = 2;
  EXPECT_NE(retrieved_config1.i, config1.i);
  // Get a copy
  auto retrieved_config_copy =
      configuration_interface.getConfig<Config1, TargetState1>();
  EXPECT_EQ(retrieved_config1.i, retrieved_config_copy.i);
  // Verify error when retrieving non-existent or not correct config types
  EXPECT_THROW((configuration_interface.getConfig<Config1, TargetState2>()),
               std::runtime_error);
  EXPECT_THROW((configuration_interface.getConfig<Config1, TargetState3>()),
               std::runtime_error);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
