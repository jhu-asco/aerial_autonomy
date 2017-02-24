#include <parsernode/common.h>
#include <parsernode/parser.h>

class SampleParser : public parsernode::Parser {
public:
  SampleParser() {}
  void setRC(int16_t channels[4]) {
    for (int i = 0; i < 4; i++) {
      quad_data.servo_in[i] = channels[i];
    }
  }
  virtual bool takeoff() {
    quad_data.quadstate = "takeoff";
    return true;
  } // Take off the quadcopter
  virtual bool land() {
    quad_data.quadstate = "land";
    return true;
  } // Land the quadcopter
  virtual bool disarm() {
    quad_data.quadstate = "disarm";
    return true;
  } // Disarm the quadcopter
  virtual bool flowControl(bool) {
    quad_data.quadstate = "flowcontrol";
    return true;
  } // Enable or disable control of quadcopter
  virtual bool calibrateimubias() {
    quad_data.quadstate = "calibrateimu";
    return true;
  } // Calibrate imu of the quadcopter based on sample data

  // Command the euler angles of the quad and the thrust. the msg format is
  // (x,y,z,w) -> (roll, pitch, yaw, thrust). The thrust value will be adjusted
  // based on whethere there is thrust bias or not.
  virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg,
                            bool sendyaw = false) {
    quad_data.rpydata.x = rpytmsg.x;
    quad_data.rpydata.y = rpytmsg.y;
    quad_data.rpydata.z = rpytmsg.z;
    return true;
  }
  virtual bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang) {
    quad_data.linvel = vel_cmd;
    quad_data.rpydata.z = yaw_ang;
    return true;
  }
  virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos,
                           double desired_yaw = 0) {
    quad_data.localpos = desired_pos;
    quad_data.rpydata.z = desired_yaw;
    return true;
  }
  virtual void grip(int state) {}
  virtual void reset_attitude(double roll, double pitch, double yaw) {}
  virtual void setmode(std::string mode) {}

  // PluginLib initialization function
  virtual void initialize(ros::NodeHandle &nh_) {}
  virtual void getquaddata(parsernode::common::quaddata &d1) { d1 = quad_data; }
  virtual ~SampleParser() {}
  virtual void
  setaltitude(double altitude_){};         // Set the altitude value in the data
  virtual void setlogdir(string logdir){}; // Set whether to log data or not
  virtual void controllog(bool logswitch) {}

private:
  parsernode::common::quaddata quad_data;
};
