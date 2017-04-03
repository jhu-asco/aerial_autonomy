#include <gtest/gtest.h>

#include <chrono>
#include <sensor_msgs/image_encodings.h>
#include <thread>

#include "aerial_autonomy/common/roi_to_position_converter.h"

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
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
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

TEST(RoiToPositionConverterTests, ComputeObjectPosition) {
  sensor_msgs::RegionOfInterest roi;
  cv::Mat depth(40, 40, CV_32F);
  sensor_msgs::CameraInfo camera_info;
  double max_distance = 3.0;
  double front_percent = 1.0;
  Position pos;

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

  RoiToPositionConverter::computeObjectPosition(
      roi, depth, camera_info, max_distance, front_percent, pos);
  ASSERT_NEAR(pos.x, (5.5 - cx) / fx, 1e-5);
  ASSERT_NEAR(pos.y, (5.5 - cy) / fy, 1e-5);
  ASSERT_NEAR(pos.z, 1, 1e-5);
}

TEST(RoiToPositionConverterTests, ComputeObjectPositionFront) {
  sensor_msgs::RegionOfInterest roi;
  cv::Mat depth(40, 40, CV_32F);
  sensor_msgs::CameraInfo camera_info;
  double max_distance = 3.0;
  double front_percent = 0.25;
  Position pos;

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

  RoiToPositionConverter::computeObjectPosition(
      roi, depth, camera_info, max_distance, front_percent, pos);
  ASSERT_NEAR(pos.x, 0.5 * (2 - cx) / fx, 1e-5);
  ASSERT_NEAR(pos.y, 0.5 * (2 - cy) / fy, 1e-5);
  ASSERT_NEAR(pos.z, 0.5, 1e-5);
}

TEST(RoiToPositionConverterTests, ComputeObjectPositionMaxDistance) {
  sensor_msgs::RegionOfInterest roi;
  cv::Mat depth(40, 40, CV_32F);
  sensor_msgs::CameraInfo camera_info;
  double max_distance = 3.0;
  double front_percent = 0.25;
  Position pos;

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

  RoiToPositionConverter::computeObjectPosition(
      roi, depth, camera_info, max_distance, front_percent, pos);
  ASSERT_NEAR(pos.x, max_distance * (0 - cx) / fx, 1e-5);
  ASSERT_NEAR(pos.y, max_distance * (0 - cy) / fy, 1e-5);
  ASSERT_NEAR(pos.z, max_distance, 1e-5);
}

TEST_F(RoiToPositionConverterROSTests, PositionValid) {
  ros::NodeHandle nh;
  RoiToPositionConverter converter(nh);

  ASSERT_FALSE(converter.positionIsValid());

  sensor_msgs::CameraInfo camera_info;
  camera_info.K[2] = 1;
  camera_info.K[5] = 1;
  camera_info.K[0] = 1;
  camera_info.K[4] = 1;
  publishCameraInfo(camera_info);
  ASSERT_FALSE(converter.positionIsValid());

  sensor_msgs::RegionOfInterest roi;
  publishRoi(roi);
  ASSERT_TRUE(converter.positionIsValid());

  /// \todo Matt This should depend on the configured ROI timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  ASSERT_FALSE(converter.positionIsValid());
  /// \todo Matt when isPositionValid depends on depth, we should publish a
  /// depth too
}

TEST_F(RoiToPositionConverterROSTests, GetPosition) {
  ros::NodeHandle nh;
  RoiToPositionConverter converter(nh);
  Position pos;

  ASSERT_FALSE(converter.getObjectPosition(pos));

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
  ASSERT_FALSE(converter.getObjectPosition(pos));

  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 10;
  roi.width = 10;
  publishRoi(roi);
  /// \todo Matt when isPositionValid depends on depth, getObjectPosition should
  /// return false when no depth has been received

  cv::Mat depth(40, 40, CV_32F);
  depth(cv::Rect(0, 0, 40, 40)).setTo(1);
  depth(cv::Rect(0, 0, 5, 5)).setTo(0.5);
  publishDepth(depth);

  ASSERT_TRUE(converter.getObjectPosition(pos));
  ASSERT_NEAR(pos.x, 0.5 * (2 - cx) / fx, 1e-5);
  ASSERT_NEAR(pos.y, 0.5 * (2 - cy) / fy, 1e-5);
  ASSERT_NEAR(pos.z, 0.5, 1e-5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "roi_to_position_converter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
