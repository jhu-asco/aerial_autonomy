#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/controllers/builtin_position_controller.h>
#include <gtest/gtest.h>
#include <parsernode/common.h>
#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>
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
    std::cout << "cmdwaypoiint" << std::endl;
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
  virtual void
  setaltitude(double altitude_){};         // Set the altitude value in the data
  virtual void setlogdir(string logdir){}; // Set whether to log data or not
  virtual void controllog(bool logswitch) {}

private:
  parsernode::common::quaddata quad_data;
};

/// \brief Test BuiltInPositionController
TEST(PositionControllerDroneConnectorTests, Constructor) {
  TestQuad tq;
  parsernode::Parser &drone_hardware = tq;

  BuiltInPositionController position_controller;

  ASSERT_NO_THROW(new PositionControllerDroneConnector(drone_hardware,
                                                       position_controller));
}

TEST(PositionControllerDroneConnectorTests, SetGoal) {
  TestQuad tq;
  parsernode::Parser &drone_hardware = tq;

  // Create controller and connector
  BuiltInPositionController position_controller;
  auto pos_controller_connector =
      new PositionControllerDroneConnector(drone_hardware, position_controller);

  // Test set goal
  PositionYaw goal(10, 10, 10, 0.1);
  pos_controller_connector->setGoal(goal);
  pos_controller_connector->run();

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  ASSERT_EQ(sensor_data.localpos.x, goal.x);
  ASSERT_EQ(sensor_data.localpos.y, goal.y);
  ASSERT_EQ(sensor_data.localpos.z, goal.z);
  ASSERT_EQ(sensor_data.rpydata.z, goal.yaw);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
