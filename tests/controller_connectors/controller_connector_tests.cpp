#include <aerial_autonomy/controller_connectors/base_controller_connector.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
class SampleController : public Controller<int, int, int> {
  virtual bool runImplementation(int, int, int &control) {
    control = 0;
    return true;
  }
  virtual ControllerStatus isConvergedImplementation(int, int) {
    return ControllerStatus(ControllerStatus::Completed);
  }
};

class SampleControllerConnector : public ControllerConnector<int, int, int> {
public:
  SampleControllerConnector(Controller<int, int, int> &controller)
      : ControllerConnector<int, int, int>(controller, ControllerGroup::Arm) {}
  virtual void sendControllerCommands(int) { return; }

  virtual bool extractSensorData(int &sensor_data) {
    sensor_data = 0;
    return true;
  }
};

////

/// \brief TEST
/// All the tests are defined here
TEST(BaseControllerConnectorTests, EmptyConnector) {
  SampleController controller;
  ASSERT_NO_THROW(new SampleControllerConnector(controller));
}

TEST(BaseControllerConnectorTests, EmptyRunFunction) {
  SampleController controller;
  SampleControllerConnector controller_connector(controller);
  ASSERT_NO_THROW(controller_connector.run());
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
