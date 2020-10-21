#include "aerial_autonomy/types/path_sensor_trajectory.h"
#include <aerial_autonomy/sensors/path_sensor.h>
#include "aerial_autonomy/tests/test_utils.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <math.h>

class PathSensorTrajectoryTests : public ::testing::Test {
public:
  PathSensorTrajectoryTests() {
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

TEST_F(PathSensorTrajectoryTests, Initialize) {
  SensorPtr<PathReturnT> goal;
  goal.reset(new PathSensor(config));
  ASSERT_NO_THROW(PathSensorTrajectory(goal));
  // ASSERT_NO_THROW( PathSensorTrajectory( path_sensor ) );
}

TEST_F(PathSensorTrajectoryTests, AtTimeTest) {
  SensorPtr<PathReturnT> path_sensor(new PathSensor(config));
  PathSensorTrajectory path_traj(path_sensor);
  //Build path_msg
  trajectory_msgs::JointTrajectory path_msg;
  path_msg.header.stamp = ros::Time::now();
  for (int ii = 0; ii < 150; ++ii) {
    trajectory_msgs::JointTrajectoryPoint point;
    //xyz
    point.positions.push_back(0.1 * ii);
    point.positions.push_back(0.2 * ii);
    point.positions.push_back(0.3 * ii);
    //rotlog
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(1.0);
    //velocity xyz
    point.velocities.push_back(0.1 * ii);
    point.velocities.push_back(0.2 * ii);
    point.velocities.push_back(0.3 * ii);
    //ang vel
    point.velocities.push_back(0.0);
    point.velocities.push_back(0.0);
    point.velocities.push_back(1.0);
    // accelerations
    point.accelerations.push_back(0.1 * ii);
    point.accelerations.push_back(0.2 * ii);
    point.accelerations.push_back(0.3 * ii);
    path_msg.points.push_back(point);
  }
  //Send path_msg
  path_pub.publish(path_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  auto state_control_pair = path_traj.atTime(ros::Time::now().toSec() + 20);//Long after the time should be up
  double x = 0.1*149;
  double y = 0.2*149;
  double z = 0.3*149;
  Eigen::VectorXd state = state_control_pair.first;
  Eigen::VectorXd control = state_control_pair.second;
  ASSERT_EQ(state.size(), 12);
  ASSERT_EQ(control.size(), 4);
  // Test state
  // pos
  ASSERT_NEAR(state(0), x, 1e-3);
  ASSERT_NEAR(state(1), y, 1e-3);
  ASSERT_NEAR(state(2), z, 1e-3);
  // rpy
  ASSERT_NEAR(state(3), 0, 1e-3);
  ASSERT_NEAR(state(4), 0, 1e-3);
  ASSERT_NEAR(state(5), 1, 1e-3);
  // pos
  ASSERT_NEAR(state(6), x, 1e-3);
  ASSERT_NEAR(state(7), y, 1e-3);
  ASSERT_NEAR(state(8), z, 1e-3);
  // rpy
  ASSERT_NEAR(state(9), 0, 1e-3);
  ASSERT_NEAR(state(10), 0, 1e-3);
  ASSERT_NEAR(state(11), 1, 1e-3);
  // Test control (roll, pitch, yaw, thrust)
  ASSERT_NEAR(control(0), sqrt(x * x + y * y + (z + 9.81) * (z + 9.81)) / 9.81,
              1e-3);
  ASSERT_NEAR(control(1), 0, 1e-3);
  ASSERT_NEAR(control(2), 0, 1e-3);
  ASSERT_NEAR(control(3), 1, 1e-3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "path_sensor_trajectory_tests");
  return RUN_ALL_TESTS();
}
