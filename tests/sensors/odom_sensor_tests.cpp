#include <aerial_autonomy/sensors/odom_sensor.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>

class OdomSensorTests : public ::testing::Test {
public:
  OdomSensorTests() {
    odom_pub = nh.advertise<nav_msgs::Odometry>("/vins_estimator/odometry", 1);
  }

  ros::NodeHandle nh;
  ros::Publisher odom_pub;
  OdomSensorConfig config;
};

using namespace quad_simulator;

TEST_F(OdomSensorTests, Constructor) {
  ASSERT_NO_THROW(new OdometrySensor(config));
}

TEST_F(OdomSensorTests, SensorTF) {
  OdomSensorConfig new_config;

  auto tf_config = new_config.mutable_sensor_transform();
  auto rot_config = tf_config->mutable_rotation();
  rot_config->set_r(-1.57);
  rot_config->set_p(0.0);
  rot_config->set_y(1.57);
  auto pos_config = tf_config->mutable_position();
  pos_config->set_x(5.0);
  pos_config->set_y(2.0);
  pos_config->set_z(1.0);

  tf::Transform sensor_tf(tf::createQuaternionFromRPY(-1.57, 0.0, 1.57),
                          tf::Vector3(5.0, 2.0, 1.0));

  OdometrySensor sensor(new_config);
  nav_msgs::Odometry odom_msg;
  tf::Vector3 pos(10, 10, 10);
  tf::Quaternion orientation(0, 0, 1, 0);
  odom_msg.pose.pose.position.x = pos.getX();
  odom_msg.pose.pose.position.y = pos.getY();
  odom_msg.pose.pose.position.z = pos.getZ();
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = 1;
  odom_msg.pose.pose.orientation.w = 0;
  tf::Vector3 linVel(0.1, -0.2, 0.3);
  tf::Vector3 angVel(0.1, -0.2, 0.3);
  odom_msg.twist.twist.linear.x = linVel.getX();
  odom_msg.twist.twist.linear.y = linVel.getY();
  odom_msg.twist.twist.linear.z = linVel.getZ();
  odom_msg.twist.twist.angular.x = angVel.getX();
  odom_msg.twist.twist.angular.y = angVel.getY();
  odom_msg.twist.twist.angular.z = angVel.getZ();
  odom_pub.publish(odom_msg);

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  tf::Vector3 actual_pos = sensor_tf * pos;
  tf::Quaternion actual_ori = sensor_tf * orientation;
  tf::Transform sensor_tf_vel(sensor_tf.getRotation());
  tf::Vector3 actual_linvel = sensor_tf_vel * linVel;
  tf::Vector3 actual_angvel = sensor_tf_vel * angVel;

  std::tuple<VelocityYawRate, PositionYaw> sensor_data = sensor.getSensorData();
  VelocityYawRate sensor_vel = std::get<0>(sensor_data);
  PositionYaw sensor_pos = std::get<1>(sensor_data);
  ASSERT_NEAR(sensor_vel.x, actual_linvel.getX(), 1e-4);
  ASSERT_NEAR(sensor_vel.y, actual_linvel.getY(), 1e-4);
  ASSERT_NEAR(sensor_vel.z, actual_linvel.getZ(), 1e-4);
  ASSERT_NEAR(sensor_vel.yaw_rate, actual_angvel.getZ(), 1e-4);
  ASSERT_NEAR(sensor_pos.x, actual_pos.getX(), 1e-4);
  ASSERT_NEAR(sensor_pos.y, actual_pos.getY(), 1e-4);
  ASSERT_NEAR(sensor_pos.z, actual_pos.getZ(), 1e-4);
  ASSERT_NEAR(sensor_pos.yaw, tf::getYaw(actual_ori), 1e-4);
}

TEST_F(OdomSensorTests, Timeout) {
  OdometrySensor sensor(config);
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.twist.twist.linear.x = 0.1;
  odom_msg.twist.twist.linear.y = -0.2;
  odom_msg.twist.twist.linear.z = 0.3;

  odom_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  ASSERT_TRUE(sensor_status_to_bool(sensor.getSensorStatus()));

  ros::Duration(1.0).sleep();
  ASSERT_FALSE(sensor_status_to_bool(sensor.getSensorStatus()));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Odom_sensor_tests");
  return RUN_ALL_TESTS();
}
