#include <gtest/gtest.h>

#include <chrono>
#include <sensor_msgs/image_encodings.h>
#include <thread>

#include "aerial_autonomy/trackers/roi_to_position_converter.h"

class RoiToPositionConverterROSTests : public ::testing::Test {
public:
  RoiToPositionConverterROSTests()
      : nh_(), camera_info_pub_(
                   nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1)),
        roi_pub_(nh_.advertise<sensor_msgs::RegionOfInterest>("roi", 1)),
        depth_pub_(nh_.advertise<sensor_msgs::Image>("depth", 1)) {}
  void publishCameraInfo(sensor_msgs::CameraInfo &camera_info) {
    camera_info_pub_.publish(camera_info);
    ros::spinOnce();
  }
  void publishRoi(sensor_msgs::RegionOfInterest &roi) {
    roi_pub_.publish(roi);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ros::spinOnce();
  }
  void publishDepth(cv::Mat &depth) {
    cv_bridge::CvImage depth_msg;
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_msg.image = depth;
    depth_pub_.publish(depth_msg.toImageMsg());
    ros::spinOnce();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher camera_info_pub_;
  ros::Publisher roi_pub_;
  ros::Publisher depth_pub_;
};

TEST(RoiToPositionConverterTests, ComputeTrackingVector) {
  RoiToPositionConverter converter("");
  sensor_msgs::RegionOfInterest roi;
  cv::Mat depth(40, 40, CV_32F);
  sensor_msgs::CameraInfo camera_info;
  double max_distance = 3.0;
  double front_percent = 1.0;
  tf::Transform pose;

  roi.x_offset = 1;
  roi.y_offset = 1;
  roi.height = 10;
  roi.width = 10;

  depth(cv::Rect(0, 0, 40, 40)).setTo(1);

  double cx = 20;
  double cy = 20;
  double fx = 2;
  double fy = 2;

  camera_info.K[2] = cx;
  camera_info.K[5] = cy;
  camera_info.K[0] = fx;
  camera_info.K[4] = fy;

  converter.computeTrackingVector(roi, depth, camera_info, max_distance,
                                  front_percent, pose);
  ASSERT_NEAR(pose.getOrigin().x(), (5.5 - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(), (5.5 - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), 1, 1e-5);
}

TEST(RoiToPositionConverterTests, ComputeTrackingVectorFront) {
  RoiToPositionConverter converter("");
  sensor_msgs::RegionOfInterest roi;
  cv::Mat depth(40, 40, CV_32F);
  sensor_msgs::CameraInfo camera_info;
  double max_distance = 3.0;
  double front_percent = 0.25;
  tf::Transform pose;

  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 10;
  roi.width = 10;

  depth(cv::Rect(0, 0, 40, 40)).setTo(1);
  depth(cv::Rect(0, 0, 5, 5)).setTo(0.5);

  double cx = 20;
  double cy = 20;
  double fx = 2;
  double fy = 2;

  camera_info.K[2] = cx;
  camera_info.K[5] = cy;
  camera_info.K[0] = fx;
  camera_info.K[4] = fy;

  converter.computeTrackingVector(roi, depth, camera_info, max_distance,
                                  front_percent, pose);
  ASSERT_NEAR(pose.getOrigin().x(), 0.5 * (2 - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(), 0.5 * (2 - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), 0.5, 1e-5);
}

TEST(RoiToPositionConverterTests, ComputeTrackingVectorMaxDistance) {
  RoiToPositionConverter converter("");
  sensor_msgs::RegionOfInterest roi;
  cv::Mat depth(40, 40, CV_32F);
  sensor_msgs::CameraInfo camera_info;
  double max_distance = 3.0;
  double front_percent = 0.25;
  tf::Transform pose;

  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 10;
  roi.width = 10;

  depth(cv::Rect(0, 0, 40, 40)).setTo(10);

  double cx = 20;
  double cy = 20;
  double fx = 2;
  double fy = 2;

  camera_info.K[2] = cx;
  camera_info.K[5] = cy;
  camera_info.K[0] = fx;
  camera_info.K[4] = fy;

  converter.computeTrackingVector(roi, depth, camera_info, max_distance,
                                  front_percent, pose);
  ASSERT_NEAR(pose.getOrigin().x(),
              max_distance * (roi.x_offset + roi.width / 2. - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(),
              max_distance * (roi.y_offset + roi.height / 2. - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), max_distance, 1e-5);
}

TEST_F(RoiToPositionConverterROSTests, TrackingValid) {
  RoiToPositionConverter converter("");
  while (!converter.isConnected()) {
  }

  ASSERT_FALSE(converter.trackingIsValid());

  sensor_msgs::CameraInfo camera_info;
  camera_info.K[2] = 1;
  camera_info.K[5] = 1;
  camera_info.K[0] = 1;
  camera_info.K[4] = 1;
  publishCameraInfo(camera_info);
  ASSERT_FALSE(converter.trackingIsValid());

  sensor_msgs::RegionOfInterest roi;
  publishRoi(roi);
  ASSERT_FALSE(converter.trackingIsValid());

  cv::Mat depth(40, 40, CV_32F);
  publishDepth(depth);
  ASSERT_TRUE(converter.trackingIsValid());

  /// \todo Matt This should depend on the configured ROI timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  ASSERT_FALSE(converter.trackingIsValid());
  /// \todo Matt when isPositionValid depends on depth, we should publish a
  /// depth too
}

TEST_F(RoiToPositionConverterROSTests, GetTrackingVector) {
  RoiToPositionConverter converter("");
  while (!converter.isConnected()) {
  }
  tf::Transform pose;

  ASSERT_FALSE(converter.getTrackingVector(pose));

  sensor_msgs::CameraInfo camera_info;
  double cx = 20;
  double cy = 20;
  double fx = 2;
  double fy = 2;
  camera_info.K[2] = cx;
  camera_info.K[5] = cy;
  camera_info.K[0] = fx;
  camera_info.K[4] = fy;
  publishCameraInfo(camera_info);
  ASSERT_FALSE(converter.getTrackingVector(pose));

  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 10;
  roi.width = 10;
  publishRoi(roi);
  /// \todo Matt when isPositionValid depends on depth, getTrackingVector should
  /// return false when no depth has been received

  cv::Mat depth(40, 40, CV_32F);
  depth(cv::Rect(0, 0, 40, 40)).setTo(1);
  depth(cv::Rect(0, 0, 5, 5)).setTo(0.5);
  publishDepth(depth);

  ASSERT_TRUE(converter.getTrackingVector(pose));
  ASSERT_NEAR(pose.getOrigin().x(), 0.5 * (2 - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(), 0.5 * (2 - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), 0.5, 1e-5);
  ASSERT_NEAR(pose.getRotation().x(), 0, 1e-5);
  ASSERT_NEAR(pose.getRotation().y(), 0, 1e-5);
  ASSERT_NEAR(pose.getRotation().z(), 0, 1e-5);
  ASSERT_NEAR(pose.getRotation().w(), 1, 1e-5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "roi_to_position_converter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
