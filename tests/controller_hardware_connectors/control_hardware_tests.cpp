#include <aerial_autonomy/controller_hardware_connectors/base_control_hardware_connector.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
class SampleController : public Controller<int, int, int> {
  virtual void setGoal(int) { return; }
  virtual int run(int) { return 0; }
};

class SampleHardwareController : ControllerHardwareConnector<int, int, int> {
public:
  SampleHardwareController(Controller<int, int, int> &ctrlr)
      : ControllerHardwareConnector<int, int, int>(ctrlr) {}
  virtual void sendHardwareCommands(int) { return; }

  virtual int extractSensorData() { return 0; }
};

////

/// \brief TEST
/// All the tests are defined here
TEST(BaseCtrlrHrdwrCnctrTests, EmptyConnector) {
  SampleController ctrlr;
  ASSERT_NO_THROW(new SampleHardwareController(ctrlr));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
