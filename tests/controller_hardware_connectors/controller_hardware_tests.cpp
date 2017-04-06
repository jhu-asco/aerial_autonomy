#include <aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
class SampleController : public Controller<int, int, int> {
  virtual int runImplementation(int, int, ControllerStatus &) { return 0; }
  virtual bool isConvergedImplementation(int, int) { return true; }
};

class SampleHardwareController
    : public ControllerHardwareConnector<int, int, int> {
public:
  SampleHardwareController(Controller<int, int, int> &controller)
      : ControllerHardwareConnector<int, int, int>(controller,
                                                   HardwareType::Arm) {}
  virtual void sendHardwareCommands(int) { return; }

  virtual int extractSensorData(ControllerStatus &) { return 0; }
};

////

/// \brief TEST
/// All the tests are defined here
TEST(BaseControllerHardwareTests, EmptyConnector) {
  SampleController controller;
  ASSERT_NO_THROW(new SampleHardwareController(controller));
}

TEST(BaseControllerHardwareTests, EmptyRunFunction) {
  SampleController controller;
  SampleHardwareController controller_connector(controller);
  ASSERT_NO_THROW(controller_connector.run());
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
