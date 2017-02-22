#include <aerial_autonomy/state_machines/basic_state_machine.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class TestQuad : public parsernode::Parser {
public:
  virtual bool takeoff() { return true; } // Take off the quadcopter
  virtual bool land() { return true; }    // Land the quadcopter
  virtual bool disarm() { return true; }  // Disarm the quadcopter
  virtual bool flowControl(bool) {
    return true;
  } // Enable or disable control of quadcopter
  virtual bool calibrateimubias() {
    return true;
  } // Calibrate imu of the quadcopter based on sample data

  // Command the euler angles of the quad and the thrust. the msg format is
  // (x,y,z,w) -> (roll, pitch, yaw, thrust). The thrust value will be adjusted
  // based on whethere there is thrust bias or not.
  virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg,
                            bool sendyaw = false) {
    return true;
  }
  virtual bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang) {
    return true;
  }
  virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos,
                           double desired_yaw = 0) {
    quad_data.localpos = desired_pos;
    quad_data.rpydata.z = desired_yaw;
    return true;
  }
  virtual void grip(int state) {}
  virtual void reset_attitude(double roll, double pitch, double yaw) {}
  virtual void setmode(std::string mode) {}

  // PluginLib initialization function
  virtual void initialize(ros::NodeHandle &nh_) {}
  virtual void getquaddata(parsernode::common::quaddata &d1) { d1 = quad_data; }
  virtual ~TestQuad() {}
  virtual void setaltitude(double) {} // Set the altitude value in the data
  virtual void setlogdir(string) {}   // Set whether to log data or not
  virtual void controllog(bool) {}

private:
  parsernode::common::quaddata quad_data;
};

/// \brief Test BuiltInPositionController
TEST(StateMachineTests, Constructor) {
  TestQuad drone_hardware;
  QuadRotorSystem quad_system(drone_hardware);
  ASSERT_NO_THROW(new LogicStateMachine(boost::ref(quad_system)));
}

TEST(StateMachineTests, ProcessEvents) {
  TestQuad drone_hardware;
  QuadRotorSystem quad_system(drone_hardware);
  LogicStateMachine logic_state_machine(boost::ref(quad_system));
  logic_state_machine.start();
  // Print initial state
  pstate(logic_state_machine);
  // Set right conditions
  logic_state_machine.process_event(Takeoff());
  // Check if we end up in right state
  // Print state
  pstate(logic_state_machine);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
