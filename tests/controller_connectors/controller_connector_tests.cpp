#include <aerial_autonomy/controller_connectors/base_controller_connector.h>
#include <aerial_autonomy/tests/sample_robot_system.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct SampleController : public Controller<int, int, int> {
  virtual bool runImplementation(int, int goal, int &control) {
    control = goal + 1;
    control_ = control;
    return true;
  }
  virtual ControllerStatus isConvergedImplementation(int, int) {
    return ControllerStatus(ControllerStatus::Completed);
  }
  int control_ = 0;
};

class LowlevelSampleControllerConnector
    : public ControllerConnector<int, int, int> {
public:
  LowlevelSampleControllerConnector(Controller<int, int, int> &controller)
      : ControllerConnector<int, int, int>(controller, ControllerGroup::UAV) {}
  virtual void sendControllerCommands(int) { return; }

  virtual bool extractSensorData(int &sensor_data) {
    sensor_data = 0;
    return true;
  }
};

class SampleControllerConnector : public ControllerConnector<int, int, int> {
public:
  SampleControllerConnector(
      Controller<int, int, int> &controller,
      LowlevelSampleControllerConnector &lowlevel_connector)
      : ControllerConnector<int, int, int>(controller, ControllerGroup::Arm),
        lowlevel_connector_(lowlevel_connector) {}
  virtual void sendControllerCommands(int control) {
    lowlevel_connector_.setGoal(control);
  }

  virtual void initialize() {
    // Set the goal of the low level controller by running the controller and
    // sending output
    // to low level connector
    run();
  }

  virtual bool extractSensorData(int &sensor_data) {
    sensor_data = 0;
    return true;
  }
  virtual AbstractControllerConnector *getDependentConnector() {
    return &lowlevel_connector_;
  }

protected:
  LowlevelSampleControllerConnector &lowlevel_connector_;
};

////

/// \brief TEST
/// All the tests are defined here
TEST(BaseControllerConnectorTests, EmptyConnector) {
  SampleController controller;
  ASSERT_NO_THROW(new LowlevelSampleControllerConnector(controller));
}

TEST(BaseControllerConnectorTests, EmptyRunFunction) {
  SampleController controller;
  LowlevelSampleControllerConnector controller_connector(controller);
  controller_connector.setGoal(2);
  ASSERT_NO_THROW(controller_connector.run());
}
TEST(BaseControllerConnectorTests, DefaultStatus) {
  SampleController controller;
  LowlevelSampleControllerConnector controller_connector(controller);
  ASSERT_EQ(controller_connector.getStatus(), ControllerStatus::NotEngaged);
}
TEST(BaseControllerConnectorTests, Disengage) {
  SampleController controller;
  LowlevelSampleControllerConnector controller_connector(controller);
  controller_connector.setGoal(2);
  ASSERT_EQ(controller_connector.getStatus(), ControllerStatus::Active);
  controller_connector.disengage();
  ASSERT_EQ(controller_connector.getStatus(), ControllerStatus::NotEngaged);
  // Running controller will not change status when disengaged
  controller_connector.run();
  ASSERT_EQ(controller_connector.getStatus(), ControllerStatus::NotEngaged);
}

///
/// \brief TEST Sample robot system
TEST(SampleRobotSystemTest, DependentConnectorActivation) {
  SampleController controller1, controller2;
  LowlevelSampleControllerConnector lowlevel_controller_connector(controller1);
  SampleControllerConnector highlevel_controller_connector(
      controller2, lowlevel_controller_connector);
  highlevel_controller_connector.setGoal(5);
  SampleRobotSystem robot_system;
  robot_system.activateControllerConnector(&highlevel_controller_connector);
  robot_system.runActiveController(ControllerGroup::UAV);
  ASSERT_EQ(lowlevel_controller_connector.getGoal(), controller2.control_);
  ASSERT_EQ(controller1.control_, controller2.control_ + 1);
}
TEST(SampleRobotSystemTest, DependentConnectorSetGoal) {
  SampleController controller1, controller2;
  LowlevelSampleControllerConnector lowlevel_controller_connector(controller1);
  SampleControllerConnector highlevel_controller_connector(
      controller2, lowlevel_controller_connector);
  SampleRobotSystem robot_system;
  robot_system.addControllerConnector(highlevel_controller_connector);
  robot_system.addControllerConnector(lowlevel_controller_connector);
  robot_system.setGoal<SampleControllerConnector>(5);
  // Verify goals
  ASSERT_EQ(lowlevel_controller_connector.getGoal(), controller2.control_);
  ASSERT_EQ(controller2.control_, highlevel_controller_connector.getGoal() + 1);
  // Check dependent controller
  robot_system.runActiveController(ControllerGroup::UAV);
  ASSERT_EQ(controller1.control_, controller2.control_ + 1);
}
TEST(SampleRobotSystemTest, DependentConnectorAbort) {
  SampleController controller1, controller2;
  LowlevelSampleControllerConnector lowlevel_controller_connector(controller1);
  SampleControllerConnector highlevel_controller_connector(
      controller2, lowlevel_controller_connector);
  SampleRobotSystem robot_system;
  robot_system.addControllerConnector(highlevel_controller_connector);
  robot_system.addControllerConnector(lowlevel_controller_connector);
  robot_system.setGoal<SampleControllerConnector>(5);
  // Verify they are active
  ASSERT_EQ(lowlevel_controller_connector.getStatus(),
            ControllerStatus::Active);
  ASSERT_EQ(highlevel_controller_connector.getStatus(),
            ControllerStatus::Completed);
  // Abort
  robot_system.abortController(ControllerGroup::Arm);
  ASSERT_EQ(lowlevel_controller_connector.getStatus(),
            ControllerStatus::NotEngaged);
  ASSERT_EQ(highlevel_controller_connector.getStatus(),
            ControllerStatus::NotEngaged);
  // Running controller does not change the control
  // Since UAV group is dependent on Arm controller group
  // i.e high level controller, the UAV controller is also
  // aborted.
  robot_system.runActiveController(ControllerGroup::UAV);
  ASSERT_EQ(controller1.control_, 0);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
