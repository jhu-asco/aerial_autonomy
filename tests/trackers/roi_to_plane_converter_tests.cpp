#include <gtest/gtest.h>

#include <chrono>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <random>
#include <tf_conversions/tf_eigen.h>

#include "aerial_autonomy/trackers/roi_to_plane_converter.h"

class RoiToPlaneConverterROSTests : public ::testing::Test {
public:
  RoiToPlaneConverterROSTests()
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

TEST(RoiToPlaneConverterTests, ComputeTrackingVector) {
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

  RoiToPlaneConverter::computeTrackingVector(
      roi, depth, camera_info, max_distance, front_percent, pose);
  ASSERT_NEAR(pose.getOrigin().x(), (5.5 - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(), (5.5 - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), 1, 1e-5);
}

TEST(RoiToPlaneConverterTests, ComputeTrackingVectorFront) {
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

  RoiToPlaneConverter::computeTrackingVector(
      roi, depth, camera_info, max_distance, front_percent, pose);
  ASSERT_NEAR(pose.getOrigin().x(), 0.5 * (2 - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(), 0.5 * (2 - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), 0.5, 1e-5);
}

TEST(RoiToPlaneConverterTests, ComputeTrackingVectorMaxDistance) {
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

  RoiToPlaneConverter::computeTrackingVector(
      roi, depth, camera_info, max_distance, front_percent, pose);
  ASSERT_NEAR(pose.getOrigin().x(),
              max_distance * (roi.x_offset + roi.width / 2. - cx) / fx, 1e-5);
  ASSERT_NEAR(pose.getOrigin().y(),
              max_distance * (roi.y_offset + roi.height / 2. - cy) / fy, 1e-5);
  ASSERT_NEAR(pose.getOrigin().z(), max_distance, 1e-5);
}

TEST_F(RoiToPlaneConverterROSTests, TrackingValid) {
  RoiToPlaneConverter converter("");
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
  ASSERT_TRUE(converter.trackingIsValid());

  /// \todo Matt This should depend on the configured ROI timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  ASSERT_FALSE(converter.trackingIsValid());
  /// \todo Matt when isPositionValid depends on depth, we should publish a
  /// depth too
}

TEST_F(RoiToPlaneConverterROSTests, GetTrackingVector) {
  RoiToPlaneConverter converter("");
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

  tf::Vector3 z_vec = pose.getBasis().getColumn(2);
  ASSERT_NEAR(z_vec.getX(), 0, 1e-5);
  ASSERT_NEAR(z_vec.getY(), 0, 1e-5);
  ASSERT_NEAR(z_vec.getZ(), 1, 1e-5);
}

TEST(RoiToPlaneConverterTests, ComputePlaneFit) {

  Eigen::Vector3d norm_vec(1, 1, 1);
  norm_vec.normalize();
  Eigen::Vector3d centroid(1, 1, 1);
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(20,0,2);
  Eigen::VectorXd y = Eigen::VectorXd::LinSpaced(20,0,2);
  Eigen::VectorXd z(x.size());
  Eigen::MatrixXd roi_point_cloud(3, x.size() * y.size());
  // Gaussian noise
  const double mean = 0.0;
  const double stddev = 0.01;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  int count = 0;
  for (int i = 0; i < x.size(); i++) {
    for (int j = 0; j < y.size(); j++) {
      // Compute z coord of the plane
      double z = -((x(i) - centroid(0)) * norm_vec(0) + (y(j) - centroid(1)) *
      norm_vec(1))/norm_vec(2) + centroid(2);
      // Add noise
      double x_noise = x(i) + dist(generator);
      double y_noise = y(j) + dist(generator);
      double z_noise = z + dist(generator);
      Eigen::Vector3d point(x_noise, y_noise, z_noise);
      roi_point_cloud.col(count) = point;
      count++;
    }
  }
  tf::Transform pose;
  RoiToPlaneConverter::computePlaneFit(roi_point_cloud, pose);
  ASSERT_NEAR(pose.getOrigin().x(), centroid(0), 1e-2);
  ASSERT_NEAR(pose.getOrigin().y(), centroid(1), 1e-2);
  ASSERT_NEAR(pose.getOrigin().z(), centroid(2), 1e-2);
  // Eigenvector correspoding to smallest eigenvalue
  tf::Vector3 z_vec = pose.getBasis().getColumn(2);
  //
  if (z_vec.getX() < 0) {
    z_vec *= -1.;
  }
  ASSERT_NEAR(z_vec.getX(), norm_vec(0), 1e-2);
  ASSERT_NEAR(z_vec.getY(), norm_vec(1), 1e-2);
  ASSERT_NEAR(z_vec.getZ(), norm_vec(2), 1e-2);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "roi_to_plane_converter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
