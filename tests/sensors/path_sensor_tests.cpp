#include <aerial_autonomy/sensors/path_sensor.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <ros/ros.h>

class PathSensorTests : public ::testing::Test {
public:
  PathSensorTests() {
    path_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/path_sensor/joint_trajectory", 1);
    config.set_final_time(3);
    auto ros_config = config.mutable_ros_sensor_config();
    ros_config->set_topic("/path_sensor/joint_trajectory");
    ros_config->set_timeout(0.5);
  }

  ros::NodeHandle nh;
  ros::Publisher path_pub;
  PathSensorConfig config;
};

using namespace quad_simulator;

TEST_F(PathSensorTests, Constructor) {
  ASSERT_NO_THROW(new PathSensor(config));
}

TEST_F(PathSensorTests, SensorTest) {

  PathSensor sensor(config);
  trajectory_msgs::JointTrajectory path_msg;
  //Build path_msg
  path_msg.header.stamp = ros::Time::now();
  for (int ii = 0; ii < 150; ++ii) {
    //xyz
    path_msg.points[ii].positions[0] = 0.1*ii;
    path_msg.points[ii].positions[1] = 0.2*ii;
    path_msg.points[ii].positions[2] = 0.3*ii;
    //rotlog
    path_msg.points[ii].positions[3] = 0.0;
    path_msg.points[ii].positions[4] = 0.0;
    path_msg.points[ii].positions[5] = 1.0;
    //velocity xyz
    path_msg.points[ii].velocities[0] = 0.1*ii;
    path_msg.points[ii].velocities[1] = 0.2*ii;
    path_msg.points[ii].velocities[2] = 0.3*ii;
    //ang vel
    path_msg.points[ii].velocities[3] = 0.0;
    path_msg.points[ii].velocities[4] = 0.0;
    path_msg.points[ii].velocities[5] = 1.0;
    //accelerations
    path_msg.points[ii].accelerations[0] = 0.1*ii;
    path_msg.points[ii].accelerations[1] = 0.2*ii;
    path_msg.points[ii].accelerations[2] = 0.3*ii;
  }
  //Send path_msg
  path_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  PathReturnT sensor_data = sensor.getSensorData();
  //Assert values are the same
  ASSERT_NEAR(std::get<1>(sensor_data),config.final_time(),1e-4);
  auto path_vec = std::get<2>(sensor_data);
  for (int ii = 0; ii < 150; ++ii) {
    Eigen::Vector3d xyz = std::get<0>(path_vec[ii]);
    ASSERT_NEAR(xyz[0], 0.1*ii, 1e-4);
    ASSERT_NEAR(xyz[1], 0.2*ii, 1e-4);
    ASSERT_NEAR(xyz[2], 0.3*ii, 1e-4);
    Eigen::Vector3d rot = std::get<1>(path_vec[ii]);
    ASSERT_NEAR(rot[0], 0.0, 1e-4);
    ASSERT_NEAR(rot[1], 0.0, 1e-4);
    ASSERT_NEAR(rot[2], 1.0, 1e-4);
    Eigen::Vector3d vel = std::get<2>(path_vec[ii]);
    ASSERT_NEAR(vel[0], 0.1*ii, 1e-4);
    ASSERT_NEAR(vel[1], 0.2*ii, 1e-4);
    ASSERT_NEAR(vel[2], 0.3*ii, 1e-4);
    Eigen::Vector3d angvel = std::get<3>(path_vec[ii]);
    ASSERT_NEAR(angvel[0], 0.0, 1e-4);
    ASSERT_NEAR(angvel[1], 0.0, 1e-4);
    ASSERT_NEAR(angvel[2], 1.0, 1e-4);
    Eigen::Vector3d acc = std::get<4>(path_vec[ii]);
    ASSERT_NEAR(acc[0], 0.1*ii, 1e-4);
    ASSERT_NEAR(acc[1], 0.2*ii, 1e-4);
    ASSERT_NEAR(acc[2], 0.3*ii, 1e-4);
  }
}

TEST_F(PathSensorTests, Timeout) {
  PathSensor sensor(config);
  trajectory_msgs::JointTrajectory path_msg;
  path_msg.header.stamp = ros::Time::now();
  //Build path_msg
  for (int ii = 0; ii < 150; ++ii) {
    //xyz
    path_msg.points[ii].positions[0] = 0.1*ii;
    path_msg.points[ii].positions[1] = 0.2*ii;
    path_msg.points[ii].positions[2] = 0.3*ii;
    //rotlog
    path_msg.points[ii].positions[3] = 0.0;
    path_msg.points[ii].positions[4] = 0.0;
    path_msg.points[ii].positions[5] = 1.0;
    //velocity xyz
    path_msg.points[ii].velocities[0] = 0.1*ii;
    path_msg.points[ii].velocities[1] = 0.2*ii;
    path_msg.points[ii].velocities[2] = 0.3*ii;
    //ang vel
    path_msg.points[ii].velocities[3] = 0.0;
    path_msg.points[ii].velocities[4] = 0.0;
    path_msg.points[ii].velocities[5] = 1.0;
  }

  path_pub.publish(path_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  ASSERT_TRUE(sensor_status_to_bool(sensor.getSensorStatus()));

  ros::Duration(1.0).sleep();
  ASSERT_FALSE(sensor_status_to_bool(sensor.getSensorStatus()));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "path_sensor_tests");
  return RUN_ALL_TESTS();
}
