#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/joystick_velocity_controller.h>
#include <aerial_autonomy/sensors/guidance.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

using namespace quad_simulator;

TEST(JoystickVelocityControllerDroneConnectorTests, Constructor) {
  std::chrono::duration<double> dt = std::chrono::milliseconds(20);
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(joystick_config, dt);
  ThrustGainEstimator thrust_gain_estimator;
  std::shared_ptr<Sensor<Velocity>> sensor;

  ASSERT_NO_THROW(new JoystickVelocityControllerDroneConnector(
      drone_hardware, controller, thrust_gain_estimator, sensor));
}

TEST(JoystickVelocityControllerDroneConnectorTests, Run) {
  std::chrono::duration<double> dt = std::chrono::milliseconds(20);
  RPYTBasedVelocityControllerConfig rpyt_config_;
  rpyt_config_.set_kp_xy(2.0);
  rpyt_config_.set_kp_z(2.0);
  rpyt_config_.set_ki_xy(0.01);
  rpyt_config_.set_ki_z(0.01);
  auto vel_ctlr_config = rpyt_config_.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();

  tolerance->set_vx(1e-3);
  tolerance->set_vy(1e-3);
  tolerance->set_vz(1e-3);
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  drone_hardware.usePerfectTime();
  std::shared_ptr<Sensor<Velocity>> sensor;
  sensor.reset(new Guidance(drone_hardware));
  JoystickVelocityController controller(joystick_config, dt);
  controller.updateRPYTConfig(rpyt_config_);

  // Set stick commands
  int16_t channels[4] = {150, 100, -150, 100};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);

  ThrustGainEstimator thrust_gain_estimator(0.2, 0.1);
  JoystickVelocityControllerDroneConnector connector(
      drone_hardware, controller, thrust_gain_estimator, sensor);

  connector.setGoal(EmptyGoal());

  auto controller_run = [&] {
    connector.run();
    return (connector.getStatus() == ControllerStatus::Completed);
  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(
      controller_run, std::chrono::seconds(1), std::chrono::milliseconds(0)));
  ASSERT_NEAR(thrust_gain_estimator.getThrustGain(), 0.16, 1e-4);

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  ASSERT_NEAR(sensor_data.linvel.x, channels[0] / 1e4, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.y, channels[1] / 1e4, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.z, channels[2] / 1e4, 1e-3);
}

class ROSSensorTests : public ::testing::Test {
public:
  ROSSensorTests() {
    odom_pub = nh.advertise<nav_msgs::Odometry>("/velocity_sensor/odometry", 1);
    auto tf_config = vel_config.mutable_sensor_transform();
    auto rot_config = tf_config->mutable_rotation();
    rot_config->set_r(-1.57);
    rot_config->set_p(0.0);
    rot_config->set_y(1.57);

    sensor_quad_tf = tf::Transform(tf::createQuaternionFromRPY(-1.57, 0, 1.57),
                                   tf::Vector3(0, 0, 0));
  }

  ros::NodeHandle nh;
  ros::Publisher odom_pub;
  VelocitySensorConfig vel_config;
  tf::Transform sensor_quad_tf;
};

TEST_F(ROSSensorTests, Constructor) {
  std::chrono::duration<double> dt = std::chrono::milliseconds(20);
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(joystick_config, dt);
  ThrustGainEstimator thrust_gain_estimator;
  std::shared_ptr<Sensor<Velocity>> sensor;
  sensor.reset(new VelocitySensor(vel_config));
  ASSERT_NO_THROW(new JoystickVelocityControllerDroneConnector(
      drone_hardware, controller, thrust_gain_estimator, sensor));
}

TEST_F(ROSSensorTests, SensorStatus) {
  std::chrono::duration<double> dt = std::chrono::milliseconds(20);
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(joystick_config, dt);
  vel_config.set_timeout(0.5);
  std::shared_ptr<Sensor<Velocity>> sensor;
  sensor.reset(new VelocitySensor(vel_config));

  int16_t channels[4] = {1500, 1000, -1500, 1000};
  drone_hardware.setRC(channels);

  ThrustGainEstimator thrust_gain_estimator(0.2, 0.1);
  JoystickVelocityControllerDroneConnector connector(
      drone_hardware, controller, thrust_gain_estimator, sensor);

  connector.setGoal(EmptyGoal());
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.twist.twist.linear.x = 0.1;
  odom_msg.twist.twist.linear.y = -0.2;
  odom_msg.twist.twist.linear.z = 0.3;

  odom_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  connector.run();
  ASSERT_TRUE(connector.getStatus() == ControllerStatus::Active);

  // Sleep for time which is more than the timeout
  ros::Duration(1.0).sleep();
  connector.run();
  ASSERT_TRUE(connector.getStatus() == ControllerStatus::Critical);
}

TEST_F(ROSSensorTests, Divergence) {
  std::chrono::duration<double> dt = std::chrono::milliseconds(20);
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(joystick_config, dt);
  vel_config.set_timeout(0.5);
  std::shared_ptr<Sensor<Velocity>> sensor;
  sensor.reset(new VelocitySensor(vel_config));

  int16_t channels[4] = {1500, 1000, -1500, 1000};
  drone_hardware.setRC(channels);

  ThrustGainEstimator thrust_gain_estimator(0.2, 0.1);
  JoystickVelocityControllerDroneConnector connector(
      drone_hardware, controller, thrust_gain_estimator, sensor);

  connector.setGoal(EmptyGoal());
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.twist.twist.linear.x = 0.1;
  odom_msg.twist.twist.linear.y = -0.2;
  odom_msg.twist.twist.linear.z = 0.3;

  odom_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  connector.run();
  ASSERT_TRUE(connector.getStatus() == ControllerStatus::Active);

  // Send diverged velocity
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.twist.twist.linear.x = 0.7;
  odom_msg.twist.twist.linear.y = -0.7;
  odom_msg.twist.twist.linear.z = 0.8;

  odom_pub.publish(odom_msg);
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  connector.run();
  ASSERT_TRUE(connector.getStatus() == ControllerStatus::Critical);
}

TEST_F(ROSSensorTests, Run) {
  std::chrono::duration<double> dt = std::chrono::milliseconds(20);
  RPYTBasedVelocityControllerConfig rpyt_config_;
  rpyt_config_.set_kp_xy(2.0);
  rpyt_config_.set_kp_z(2.0);
  rpyt_config_.set_ki_xy(0.01);
  rpyt_config_.set_ki_z(0.01);
  auto vel_ctlr_config = rpyt_config_.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();

  tolerance->set_vx(1e-3);
  tolerance->set_vy(1e-3);
  tolerance->set_vz(1e-3);
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  drone_hardware.usePerfectTime();
  std::shared_ptr<Sensor<Velocity>> sensor;
  sensor.reset(new VelocitySensor(vel_config));
  JoystickVelocityController controller(joystick_config, dt);
  controller.updateRPYTConfig(rpyt_config_);

  // Set stick commands
  int16_t channels[4] = {1500, 1000, -1500, 1000};
  drone_hardware.setRC(channels);

  ThrustGainEstimator thrust_gain_estimator(0.2, 0.1);
  JoystickVelocityControllerDroneConnector connector(
      drone_hardware, controller, thrust_gain_estimator, sensor);

  connector.setGoal(EmptyGoal());

  auto controller_run = [&] {
    parsernode::common::quaddata sensor_data;
    drone_hardware.getquaddata(sensor_data);
    tf::Vector3 quad_vel = tf::Vector3(
        sensor_data.linvel.x, sensor_data.linvel.y, sensor_data.linvel.z);
    tf::Vector3 sensor_vel = sensor_quad_tf.inverse() * quad_vel;
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.twist.twist.linear.x = sensor_vel[0];
    odom_msg.twist.twist.linear.y = sensor_vel[1];
    odom_msg.twist.twist.linear.z = sensor_vel[2];
    odom_pub.publish(odom_msg);
    ros::Duration(0.001).sleep();
    ros::spinOnce();
    connector.run();
    return (connector.getStatus() == ControllerStatus::Completed);
  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(
      controller_run, std::chrono::seconds(1), std::chrono::milliseconds(0)));

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  ASSERT_NEAR(sensor_data.linvel.x, channels[0] / 1e4, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.y, channels[1] / 1e4, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.z, channels[2] / 1e4, 1e-3);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_velocity_controller_drone_connector_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
