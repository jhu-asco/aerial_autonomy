#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/joysticks_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "manual_velocity_controller_drone_connector_config.pb.h"

#include <parsernode/parser.h>
#include "aerial_autonomy/common/atomic.h"

/**
* @brief Maps Joystick goals to rpythrust commands to quadrotor
*/
class ManualVelocityControllerDroneConnector
: public ControllerHardwareConnector<std::tuple<JoysticksYaw, VelocityYaw>,
EmptyGoal,
RollPitchYawThrust> {
public:
  /**
  * @brief Constructor
  *
  * Store drone hardware with hardware type as UAV.
  * Uses parsernode::Parser::cmdrpythrust function.
  *
  * @param drone_hardware Drone hardware used to send commands
  * @param controller RpyThrust controller
  */
  ManualVelocityControllerDroneConnector(
    parsernode::Parser &drone_hardware,
    Controller<std::tuple<JoysticksYaw,VelocityYaw>, EmptyGoal, RollPitchYawThrust> &controller)
  : ControllerHardwareConnector(controller, HardwareType::UAV),
  drone_hardware_(drone_hardware),
  config_(ManualVelocityControllerDroneConnectorConfig()) {

    sensor_sub = nh_.subscribe("/rpyt_vins_connector/pose",1000,
      &ManualVelocityControllerDroneConnector::sensorCallback,this);

    sensor_tf.setOrigin(tf::Vector3(config_.sensor_tx(), 
      config_.sensor_ty(), config_.sensor_tz()));

    sensor_tf.setRotation(tf::createQuaternionFromRPY(config_.sensor_r(),
      config_.sensor_p(),
      config_.sensor_y()));
  }

protected:
  /**
   * @brief Extracts joystick commands and current yaw from hardware
   * and velocity from sensor
   *
   * @param sensor_data Joystick commands and current yaw
   *
   * @return true if succesfully extracted joystick data
   */
  virtual bool extractSensorData(std::tuple<JoysticksYaw, VelocityYaw> &sensor_data);

  /**
   * @brief  Send RPYT commands to hardware
   *
   * @param controls RPYT command to send to drone
   */
  virtual void sendHardwareCommands(RollPitchYawThrust controls);
  /**
  *
  * @brief Callback function for sensor_sub
  *
  */
  void sensorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief Config for controller connector 
  */
  ManualVelocityControllerDroneConnectorConfig config_;
  /**
  * @brief Internal controller to convert velocity commands to rpyt  
  */
  RPYTBasedVelocityController rpyt_vel_ctlr;
  /**
  *  @brief nodehandle to get sensor data
  */
  ros::NodeHandle nh_;
  /**
  * @brief subscriber to get sensor data from ros
  */
  ros::Subscriber sensor_sub;
  /**
  * @ brief variable to store velocity data
  */
  Atomic<VelocityYaw> vel_sensor_data_;
  /**
  * @ brief variable to store last position state
  */
  PositionYaw last_pos;
  /**
  * @brief Transform of sensor in robot frame
  */ 
  tf::Transform sensor_tf;
};
