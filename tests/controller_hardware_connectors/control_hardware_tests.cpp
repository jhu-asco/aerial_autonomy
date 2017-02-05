#include <aerial_autonomy/controller_hardware_connectors/base_control_hardware_connector.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
class SampleCtrlr : public Controller<int, int, int> {
  virtual void setGoal(int) { return; }
  virtual int run(int) { return 0; }
};

class SampleHrdwrCntrlr : ControllerHardwareConnector<int, int, int> {
public:
  SampleHrdwrCntrlr(Controller<int, int, int> &ctrlr)
      : ControllerHardwareConnector<int, int, int>(ctrlr) {}
  virtual void sendHardwareCommands(int) { return; }

  virtual int extractSensorData() { return 0; }
};

////

/// \brief TEST
/// All the tests are defined here
TEST(BaseCtrlrHrdwrCnctrTests, EmptyConnector) {
  SampleCtrlr ctrlr;
  ASSERT_NO_THROW(new SampleHrdwrCntrlr(ctrlr));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
