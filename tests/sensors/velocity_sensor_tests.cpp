#include <aerial_autonomy/sensors/velocity_sensor.h>
#include <aerial_autonomy/types/velocity_yaw.h>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>
#include <thread>

class VelocitySensorTests : public ::testing::Test {
public:
  VelocitySensorTests() {
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    for (int i = 0; i < 6; ++i) {
      config.add_sensor_transform(0.0);
    }
  }

  void publishData(geometry_msgs::PoseStamped pose) {
    pose_pub.publish(pose);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  ros::NodeHandle nh;
  ros::Publisher pose_pub;
  quad_simulator::QuadSimulator drone_hardware;
  VelocitySensorConfig config;
};

using namespace quad_simulator;

TEST_F(VelocitySensorTests, Constructor) {
  ASSERT_NO_THROW(new VelocitySensor(drone_hardware, nh, config));
}

TEST_F(VelocitySensorTests, StartingDataValid) {
  VelocitySensor sensor(drone_hardware, nh, config);
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = 0.1;
  pose.pose.position.y = -0.1;
  pose.pose.position.z = 0.2;
  pose.pose.orientation.w = 1.0;
  publishData(pose);
  ASSERT_TRUE(bool(sensor.getSensorStatus()));
}

TEST_F(VelocitySensorTests, StartingDataInvalid) {
  VelocitySensor sensor(drone_hardware, nh, config);
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = 0.6;
  pose.pose.position.y = -0.1;
  pose.pose.position.z = 0.2;
  pose.pose.orientation.w = 1.0;
  publishData(pose);
  ASSERT_FALSE(bool(sensor.getSensorStatus()));
}

TEST_F(VelocitySensorTests, SensorValue) {
  VelocitySensor sensor(drone_hardware, nh, config);
  ros::Time t0 = ros::Time::now();
  ros::Duration(0.03).sleep();
  ros::Time t1 = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = t0;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  publishData(pose);

  pose.header.stamp = t1;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.3;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0.1);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  publishData(pose);

  double dt = (t1 - t0).toSec();
  VelocityYaw velocity = sensor.getSensorData();
  ASSERT_NEAR(velocity.x, 0.1 / dt, 1e-4);
  ASSERT_NEAR(velocity.y, -0.2 / dt, 1e-4);
  ASSERT_NEAR(velocity.z, 0.3 / dt, 1e-4);
  ASSERT_NEAR(velocity.yaw, 0.1, 1e-4);
}

TEST_F(VelocitySensorTests, SensorStatusInvalid) {
  VelocitySensor sensor(drone_hardware, nh, config);
  geometry_msgs::PoseStamped pose;
  for (int i = 0; i > 40; ++i) {
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0.6;
    pose.pose.position.y = -0.1;
    pose.pose.position.z = 0.2;
    pose.pose.orientation.w = 1.0;
    publishData(pose);
    ros::Duration(0.03).sleep();
  }
  ASSERT_FALSE(bool(sensor.getSensorStatus()));
}

TEST_F(VelocitySensorTests, SensorTF) {
  VelocitySensorConfig new_config;

  new_config.add_sensor_transform(0.1);
  new_config.add_sensor_transform(0.0);
  new_config.add_sensor_transform(0.1);
  new_config.add_sensor_transform(-1.57);
  new_config.add_sensor_transform(0.0);
  new_config.add_sensor_transform(0.0);

  tf::Transform sensor_tf;
  sensor_tf.setOrigin(tf::Vector3(0.1, 0, 0.1));
  sensor_tf.setRotation(tf::createQuaternionFromRPY(-1.57, 0.0, 0.0));

  VelocitySensor sensor(drone_hardware, nh, new_config);
  ros::Time t0 = ros::Time::now();
  ros::Duration(0.03).sleep();
  ros::Time t1 = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = t0;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  publishData(pose);

  pose.header.stamp = t1;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.3;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  publishData(pose);

  double dt = (t1 - t0).toSec();

  VelocityYaw vel = sensor.getSensorData();
  tf::Vector3 velocity = tf::Vector3(vel.x, vel.y, vel.z);
  tf::Transform sensor_data_tf;
  sensor_data_tf.setOrigin(velocity);
  sensor_data_tf.setRotation(q);

  // sensor_frame_tf is current sensor frame in sensor's origin frame
  // which should be close to the data we published
  tf::Transform sensor_frame_tf =
      sensor_tf.inverse() * sensor_data_tf * sensor_tf;
  tf::Vector3 sensor_frame_pos = sensor_frame_tf.getOrigin();

  ASSERT_NEAR(sensor_frame_pos[0], 0.1 / dt, 1e-4);
  ASSERT_NEAR(sensor_frame_pos[1], -0.2 / dt, 1e-4);
  ASSERT_NEAR(sensor_frame_pos[2], 0.3 / dt, 1e-4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "velocity_sensor_tests");
  return RUN_ALL_TESTS();
}
