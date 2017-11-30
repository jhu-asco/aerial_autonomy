#include <aerial_autonomy/sensors/velocity_sensor.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>

class VelocitySensorTests : public ::testing::Test {
public:
  VelocitySensorTests() {
    odom_pub = nh.advertise<nav_msgs::Odometry>("/velocity_sensor/odometry", 1);
  }

  ros::NodeHandle nh;
  ros::Publisher odom_pub;
  VelocitySensorConfig config;
};

using namespace quad_simulator;

TEST_F(VelocitySensorTests, Constructor) {
  ASSERT_NO_THROW(new VelocitySensor(config));
}

TEST_F(VelocitySensorTests, SensorTF) {
  VelocitySensorConfig new_config;

  auto tf_config = new_config.mutable_sensor_transform();
  auto rot_config = tf_config->mutable_rotation();
  rot_config->set_r(-1.57);
  rot_config->set_p(0.0);
  rot_config->set_y(1.57);

  tf::Transform sensor_tf(tf::createQuaternionFromRPY(-1.57, 0.0, 1.57),
                          tf::Vector3(0.0, 0, 0.0));

  VelocitySensor sensor(new_config);
  nav_msgs::Odometry odom_msg;
  odom_msg.twist.twist.linear.x = 0.1;
  odom_msg.twist.twist.linear.y = -0.2;
  odom_msg.twist.twist.linear.z = 0.3;

  odom_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  Velocity sensor_vel = sensor.getSensorData();
  tf::Vector3 actual_vel = sensor_tf * tf::Vector3(0.1, -0.2, 0.3);
  ASSERT_NEAR(sensor_vel.x, actual_vel[0], 1e-4);
  ASSERT_NEAR(sensor_vel.y, actual_vel[1], 1e-4);
  ASSERT_NEAR(sensor_vel.z, actual_vel[2], 1e-4);
}

TEST_F(VelocitySensorTests, Timeout) {
  VelocitySensor sensor(config);
  nav_msgs::Odometry odom_msg;
  odom_msg.twist.twist.linear.x = 0.1;
  odom_msg.twist.twist.linear.y = -0.2;
  odom_msg.twist.twist.linear.z = 0.3;

  odom_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  ros::Duration(1.0).sleep();
  ASSERT_FALSE(sensor_status_to_bool(sensor.getSensorStatus()));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "velocity_sensor_tests");
  return RUN_ALL_TESTS();
}