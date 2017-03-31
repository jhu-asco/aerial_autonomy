#include <gtest/gtest.h>

#include "aerial_autonomy/common/roi_to_position_converter.h"

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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
