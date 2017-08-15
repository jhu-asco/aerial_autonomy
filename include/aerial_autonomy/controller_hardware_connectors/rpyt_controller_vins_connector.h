#pragma once
#include "ros/ros.h" 
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include <parsernode/parser.h>

/**
*
* @class RPYTControllerVINSConnector
*
* @brief Takes sensor data from VINS sensor to give rpyt commands to a drone 
*
**/
class RPYTControllerVINSConnector 
: public ControllerHardwareConnector<std::tuple<PositionYaw, VelocityYaw>, 
PositionYaw, RollPitchYawThrust>
{
public:
  /**
  * @brief Constructor
  *
  * Store hardware type as UAV.
  *
  * @param controller RPYT controller that achieves desired postion, yaw
  * 
  */
  RPYTControllerVINSConnector(
    parsernode::Parser &drone_hardware,
    Controller<std::tuple<PositionYaw, VelocityYaw>,PositionYaw,RollPitchYawThrust> &controller)
  : ControllerHardwareConnector(controller, HardwareType::UAV), 
  drone_hardware_(drone_hardware)
  {
    sensor_sub = nh_.subscribe("/rpyt_vins_connector/pose",1000,
      &RPYTControllerVINSConnector::sensorCallback,this);

    sensor_tf.setOrigin(tf::Vector3(0,0,0));
    sensor_tf.setRotation(tf::createQuaternionFromRPY(M_PI/2, 0, 0));
  }

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @param sensor_data Current position.velocity and yaw of UAV
   *
   * @return true if position.velocity and yaw can be extracted
   */
  virtual bool extractSensorData(std::tuple<PositionYaw, VelocityYaw> &sensor_data);

  /**
   * @brief  Send rpyt commands to hardware
   *
   * @param controls rpyt command to send to UAV
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
  * @brief nodehandle to get sensor data from ros
  */
  ros::NodeHandle nh_;
  /**
  * @brief subscriber to get sensor data from ros
  */
  ros::Subscriber sensor_sub;
  /*
  *@ brief variable to store sensor data 
  */
  std::tuple<PositionYaw, VelocityYaw> sensor_data_;
  /*
  * @brief transfrom of sensor in robot frame
  */
  tf::Transform sensor_tf;
};
