#include <aerial_autonomy/sensors/base_sensor.h>
#include <aerial_autonomy/sensors/velocity_sensor.h>
#include <aerial_autonomy/tests/test_utils.h>
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
  geometry_msgs::Vector3 quad_pos;
  quad_pos.x = 0.1;
  quad_pos.y = -0.2;
  quad_pos.z = 0.3;
  drone_hardware.cmdwaypoint(quad_pos);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = 0.2;
  pose.pose.position.y = -0.3;
  pose.pose.position.z = 0.1;
  pose.pose.orientation.w = 1.0;
  pose_pub.publish(pose);

  auto sensor_status_check = [&]() {
    ros::spinOnce();
    return sensor_status_to_bool(sensor.getSensorStatus());
  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(sensor_status_check,
                                          std::chrono::seconds(20)));
}

TEST_F(VelocitySensorTests, StartingDataInvalid) {
  VelocitySensor sensor(drone_hardware, nh, config);

  geometry_msgs::Vector3 quad_pos;
  quad_pos.x = 0.1;
  quad_pos.y = -0.2;
  quad_pos.z = 0.3;
  drone_hardware.cmdwaypoint(quad_pos);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = 0.7;
  pose.pose.position.y = -0.1;
  pose.pose.position.z = 0.2;
  pose.pose.orientation.w = 1.0;
  pose_pub.publish(pose);

  auto sensor_status_check = [&]() {
    ros::spinOnce();
    return sensor_status_to_bool(sensor.getSensorStatus());
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(sensor_status_check,
                                            std::chrono::seconds(20)));
}

TEST_F(VelocitySensorTests, SensorValue) {
  VelocitySensor sensor(drone_hardware, nh, config);
  ros::Time t0 = ros::Time::now();
  ros::Time t1 = t0 + ros::Duration(0.03);
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = t0;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  pose_pub.publish(pose);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  pose.header.stamp = t1;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.3;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0.1);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  pose_pub.publish(pose);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  VelocityYaw velocity = sensor.getSensorData();
  ASSERT_NEAR(velocity.x, 0.1 / 0.03, 1e-4);
  ASSERT_NEAR(velocity.y, -0.2 / 0.03, 1e-4);
  ASSERT_NEAR(velocity.z, 0.3 / 0.03, 1e-4);
  ASSERT_NEAR(velocity.yaw, 0.1, 1e-4);
}

TEST_F(VelocitySensorTests, SensorStatusInvalid) {
  VelocitySensor sensor(drone_hardware, nh, config);
  geometry_msgs::Vector3 quad_pos;
  quad_pos.x = 0.1;
  quad_pos.y = -0.2;
  quad_pos.z = 0.3;
  drone_hardware.cmdwaypoint(quad_pos);

  geometry_msgs::PoseStamped pose;
  for (int i = 0; i < 40; ++i) {
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0.1;
    pose.pose.position.y = -0.7;
    pose.pose.position.z = 1.0;
    pose.pose.orientation.w = 1.0;
    pose_pub.publish(pose);
    ros::Duration(0.03).sleep();
  }

  auto sensor_status_check = [&]() {
    ros::spinOnce();
    return sensor_status_to_bool(sensor.getSensorStatus());
  };
  ASSERT_FALSE(test_utils::waitUntilFalse()(sensor_status_check,
                                            std::chrono::seconds(20)));
}

TEST_F(VelocitySensorTests, SensorTF) {
  VelocitySensorConfig new_config;

  new_config.add_sensor_transform(0.1);
  new_config.add_sensor_transform(0.0);
  new_config.add_sensor_transform(0.1);
  new_config.add_sensor_transform(-1.57);
  new_config.add_sensor_transform(0.0);
  new_config.add_sensor_transform(0.0);

  tf::Transform sensor_tf(tf::createQuaternionFromRPY(-1.57, 0.0, 0.0),
                          tf::Vector3(0.1, 0, 0.1));

  VelocitySensor sensor(drone_hardware, nh, new_config);
  ros::Time t0 = ros::Time::now();
  ros::Time t1 = t0 + ros::Duration(0.03);
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = t0;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose_pub.publish(pose);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  pose.header.stamp = t1;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.3;
  pose_pub.publish(pose);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

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

  ASSERT_NEAR(sensor_frame_pos[0], 0.1 / 0.03, 1e-4);
  ASSERT_NEAR(sensor_frame_pos[1], -0.2 / 0.03, 1e-4);
  ASSERT_NEAR(sensor_frame_pos[2], 0.3 / 0.03, 1e-4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "velocity_sensor_tests");
  return RUN_ALL_TESTS();
}
