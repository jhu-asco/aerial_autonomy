#include<aerial_autonomy/log/log.h>
#include <aerial_autonomy/log/mocap_logger.h>

MocapLogger::MocapLogger() : nh_("mocap_log") {
  mocap_sub_ = nh_.subscribe("quad_pose_mocap", 1, &MocapLogger::logData, this);
  DATA_HEADER("mocap_logger") << "X"<< "Y" << "Z" << "Qx" << "Qy" << "Qz" << "Qw"<<DataStream::endl;
}

void MocapLogger::logData(const geometry_msgs::TransformStampedConstPtr data) {
  auto &transform = data->transform;
  DATA_LOG("mocap_logger") << transform.translation.x << transform.translation.y << transform.translation.z << transform.rotation.x << transform.rotation.y << transform.rotation.z << transform.rotation.w << DataStream::endl;
}


