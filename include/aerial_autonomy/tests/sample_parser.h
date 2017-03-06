#pragma once

#include <parsernode/common.h>
#include <parsernode/parser.h>

/**
* @brief An example UAV hardware
* that emulates actual UAV hardware
*/
class SampleParser : public parsernode::Parser {
public:
  /**
  * @brief Implict constructor
  */
  SampleParser() {}
  /**
  * @brief set the rc channel information stored
  * in the UAV brain
  *
  * @param channels rc channel values (-10000, 10000)
  */
  void setRC(int16_t channels[4]) {
    for (int i = 0; i < 4; i++) {
      quad_data.servo_in[i] = channels[i];
    }
  }
  /**
  * @brief Takeoff
  *
  * @return True if successful
  */
  virtual bool takeoff() {
    quad_data.quadstate = "takeoff";
    return true;
  }
  /**
  * @brief Land
  *
  * @return True if landing successful
  */
  virtual bool land() {
    quad_data.quadstate = "land";
    return true;
  }
  /**
  * @brief Turn off motor
  *
  * @return True if disabling is successful
  */
  virtual bool disarm() {
    quad_data.quadstate = "disarm";
    return true;
  }
  /**
  * @brief Toggle software control
  *
  * @return True if successful
  */
  virtual bool flowControl(bool) {
    quad_data.quadstate = "flowcontrol";
    return true;
  }
  /**
  * @brief Recalibrate IMU
  *
  * @return true if calibration successful
  */
  virtual bool calibrateimubias() {
    quad_data.quadstate = "calibrateimu";
    return true;
  }
  /**
  * @brief Command the euler angles of the quad and the thrust.
  *
  * @param rpytmsg Msg format is (x,y,z,w) -> (roll, pitch, yaw, thrust).
  * The thrust value will be adjusted based on whethere there is thrust bias or
  * not.
  * @param sendyaw True to control the yaw/ False to ignore yaw command
  *
  * @return True if sending command is successful
  */
  virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg,
                            bool sendyaw = false) {
    quad_data.rpydata.x = rpytmsg.x;
    quad_data.rpydata.y = rpytmsg.y;
    quad_data.rpydata.z = rpytmsg.z;
    return true;
  }
  /**
  * @brief Command velocity and yaw to UAV
  *
  * @param vel_cmd Velocity vector (m/s)
  * @param yaw_ang Yaw angle (rad)
  *
  * @return True if sending command is successful
  */
  virtual bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang) {
    quad_data.linvel = vel_cmd;
    quad_data.rpydata.z = yaw_ang;
    return true;
  }
  /**
  * @brief Command position yaw for UAV
  *
  * @param desired_pos Desired 3D position (m)
  * @param desired_yaw Desired yaw angle (rad)
  *
  * @return True if sending command is successful
  */
  virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos,
                           double desired_yaw = 0) {
    quad_data.localpos = desired_pos;
    quad_data.rpydata.z = desired_yaw;
    return true;
  }
  /**
  * @brief Toggle gripper
  *
  * @param state 0 to close, 1 to open
  */
  virtual void grip(int state) {}
  /**
  * @brief Set the attitude data to specified roll, pitch, yaw (rad)
  *
  * @param roll Roll angle (rad)
  * @param pitch Pitch angle (rad)
  * @param yaw Yaw angle (rad)
  */
  virtual void reset_attitude(double roll, double pitch, double yaw) {}
  /**
  * @brief Set UAV controller mode such position controlling, rpyt etc
  *
  * @param mode This version does not support any modes
  */
  virtual void setmode(std::string mode) {}

  // PluginLib initialization function
  /**
  * @brief Initialize the Hardware with a ros node handle to setup
  * communications
  *
  * @param nh_ Nodehandle to setup communications
  */
  virtual void initialize(ros::NodeHandle &nh_) {}
  /**
  * @brief Get the sensor data and UAV state
  *
  * @param d1 Data struct in which data is filled
  */
  virtual void getquaddata(parsernode::common::quaddata &d1) { d1 = quad_data; }
  /**
  * @brief Polymorphic destructor
  */
  virtual ~SampleParser() {}
  /**
  * @brief Set the battery percent sensor data
  *
  * @param percent
  */
  virtual void setBatteryPercent(double percent) {
    quad_data.batterypercent = percent;
  }
  /**
  * @brief set the altitude or local z height data(m)
  *
  * @param altitude_ Altitude(m)
  */
  virtual void setaltitude(double altitude_) {
    quad_data.altitude = altitude_;
    quad_data.localpos.z = altitude_;
  }
  /**
  * @brief Create log directory and create log files
  *
  * @param logdir the directory where to create log files
  */
  virtual void setlogdir(string logdir) {}
  /**
  * @brief Toggle logging
  *
  * @param logswitch True to start logging/False to stop logging
  */
  virtual void controllog(bool logswitch) {}

private:
  /**
  * @brief Sensor data and quad state
  */
  parsernode::common::quaddata quad_data;
};
