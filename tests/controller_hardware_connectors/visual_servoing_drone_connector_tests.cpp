#include <gtest/gtest.h>

#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"

#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

class VisualServoingControllerDroneConnectorTests : public ::testing::Test {
public:
  VisualServoingControllerDroneConnectorTests() : nh_(),
    roi_to_position_converter_(nh_),
    visual_servoing_connector_(new VisualServoingControllerDroneConnector(
        roi_to_position_converter_, drone_hardware_, controller_, config_)) {
  }
  ros::NodeHandle nh_;
  RoiToPositionConverter roi_to_position_converter_;
  QuadSimulator drone_hardware_;
  VisualServoingControllerConnectorConfig config_;
  ConstantHeadingDepthController controller_;
  std::unique_ptr<VisualServoingControllerDroneConnector>
      visual_servoing_connector_;
};

TEST_F(VisualServoingControllerDroneConnectorTests, Constructor) {}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "visual_servoing_drone_connector_tests");
  return RUN_ALL_TESTS();
}
