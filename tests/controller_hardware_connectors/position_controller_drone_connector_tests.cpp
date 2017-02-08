#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/controllers/builtin_position_controller.h>
#include <pluginlib/class_loader.h>
#include <parsernode/parser.h>
#include <gtest/gtest.h>

/// \brief Test BuiltInPositionController
TEST(PositionControllerDroneConnectorTests, Constructor) {
  pluginlib::ClassLoader<parsernode::Parser>* parser_loader = 
    new pluginlib::ClassLoader<parsernode::Parser>("parsernode","parsernode::Parser");

  parsernode::Parser* drone_hardware = 
    parser_loader->createInstance("quad_simulator_parser/QuadSimParser").get();

  BuiltInPositionController position_controller;

  ASSERT_NO_THROW(new PositionControllerDroneConnector(*drone_hardware, position_controller));
}

TEST(PositionControllerDroneConnectorTests, SetGoal) {
  pluginlib::ClassLoader<parsernode::Parser> *parser_loader =
      new pluginlib::ClassLoader<parsernode::Parser>("parsernode",
                                                     "parsernode::Parser");

  parsernode::Parser *drone_hardware =
      parser_loader->createInstance("quad_simulator_parser/QuadSimParser")
          .get();

  BuiltInPositionController position_controller;

  auto pos_controller_connector = new PositionControllerDroneConnector(
      *drone_hardware, position_controller);

  pos_controller_connector->setGoal(PositionYaw(10, 10, 10, 0.1));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
