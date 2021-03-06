#include <gtest/gtest.h>

#include <chrono>
#include <random>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <thread>

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
  RoiToPlaneConverter converter("");
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

TEST(RoiToPlaneConverterTests, ComputeTrackingVectorFront) {
  RoiToPlaneConverter converter("");
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

TEST(RoiToPlaneConverterTests, ComputeTrackingVectorMaxDistance) {
  RoiToPlaneConverter converter("");
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

  tf::Vector3 x_axis = pose.getBasis().getColumn(0);
  ASSERT_NEAR(x_axis.getX(), 0, 1e-5);
  ASSERT_NEAR(x_axis.getY(), 0, 1e-5);
  ASSERT_NEAR(x_axis.getZ(), -1, 1e-5);
}

TEST(RoiToPlaneConverterTests, ComputePlaneFit) {
  Eigen::Vector3d norm_vec(0, 0, -1);
  norm_vec.normalize();
  Eigen::Vector3d centroid(1, 1, 1);
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(20, 0, 2);
  Eigen::VectorXd y = Eigen::VectorXd::LinSpaced(20, 0, 2);
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
      double z = -((x(i) - centroid(0)) * norm_vec(0) +
                   (y(j) - centroid(1)) * norm_vec(1)) /
                     norm_vec(2) +
                 centroid(2);
      // Add noise
      double x_noise = x(i) + dist(generator);
      double y_noise = y(j) + dist(generator);
      double z_noise = z + dist(generator);
      Eigen::Vector3d point(x_noise, y_noise, z_noise);
      roi_point_cloud.col(count) = point;
      count++;
    }
  }
  RoiToPlaneConverter converter("");
  tf::Transform pose;
  converter.computePlaneFit(roi_point_cloud, pose);
  ASSERT_NEAR(pose.getOrigin().x(), centroid(0), 1e-2);
  ASSERT_NEAR(pose.getOrigin().y(), centroid(1), 1e-2);
  ASSERT_NEAR(pose.getOrigin().z(), centroid(2), 1e-2);
  // Eigenvector correspoding to smallest eigenvalue
  tf::Vector3 x_axis = pose.getBasis().getColumn(0);
  tf::Vector3 y_axis = pose.getBasis().getColumn(1);
  tf::Vector3 z_axis = pose.getBasis().getColumn(2);
  ASSERT_NEAR(x_axis.getX(), 0, 1e-2);
  ASSERT_NEAR(x_axis.getY(), 0, 1e-2);
  ASSERT_NEAR(x_axis.getZ(), -1, 1e-2);
  ASSERT_NEAR(y_axis.getX(), 1, 1e-2);
  ASSERT_NEAR(y_axis.getY(), 0, 1e-2);
  ASSERT_NEAR(y_axis.getZ(), 0, 1e-2);
  ASSERT_NEAR(z_axis.getX(), 0, 1e-2);
  ASSERT_NEAR(z_axis.getY(), -1, 1e-2);
  ASSERT_NEAR(z_axis.getZ(), 0, 1e-2);
}

TEST(RoiToPlaneConverterTests, ComputePlaneFitNoneZeroNormalVector) {
  Eigen::Vector3d norm_vec(0.1, 0.1, -1);
  norm_vec.normalize();
  Eigen::Vector3d centroid(1, 1, 1);
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(20, 0, 2);
  Eigen::VectorXd y = Eigen::VectorXd::LinSpaced(20, 0, 2);
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
      double z = -((x(i) - centroid(0)) * norm_vec(0) +
                   (y(j) - centroid(1)) * norm_vec(1)) /
                     norm_vec(2) +
                 centroid(2);
      // Add noise
      double x_noise = x(i) + dist(generator);
      double y_noise = y(j) + dist(generator);
      double z_noise = z + dist(generator);
      Eigen::Vector3d point(x_noise, y_noise, z_noise);
      roi_point_cloud.col(count) = point;
      count++;
    }
  }
  RoiToPlaneConverter converter("");
  tf::Transform pose;
  converter.computePlaneFit(roi_point_cloud, pose);
  ASSERT_NEAR(pose.getOrigin().x(), centroid(0), 1e-2);
  ASSERT_NEAR(pose.getOrigin().y(), centroid(1), 1e-2);
  ASSERT_NEAR(pose.getOrigin().z(), centroid(2), 1e-2);
  // Eigenvector correspoding to smallest eigenvalue
  tf::Vector3 x_axis = pose.getBasis().getColumn(0);
  ASSERT_NEAR(x_axis.getX(), norm_vec(0), 1e-2);
  ASSERT_NEAR(x_axis.getY(), norm_vec(1), 1e-2);
  ASSERT_NEAR(x_axis.getZ(), norm_vec(2), 1e-2);

  Eigen::Vector3d z_axis(0, norm_vec(2), -norm_vec(1));
  z_axis.normalize();
  Eigen::Vector3d y_axis = z_axis.cross(norm_vec);
  y_axis.normalize();
  tf::Vector3 z_axis_tf = pose.getBasis().getColumn(2);
  Eigen::Vector3d z_axis_from_pos(z_axis_tf.getX(), z_axis_tf.getY(),
                                  z_axis_tf.getZ());
  tf::Vector3 y_axis_tf = pose.getBasis().getColumn(1);
  Eigen::Vector3d y_axis_from_pos(y_axis_tf.getX(), y_axis_tf.getY(),
                                  y_axis_tf.getZ());
  ASSERT_NEAR(y_axis(0), y_axis_from_pos(0), 1e-2);
  ASSERT_NEAR(y_axis(1), y_axis_from_pos(1), 1e-2);
  ASSERT_NEAR(y_axis(2), y_axis_from_pos(2), 1e-2);
  ASSERT_NEAR(z_axis(0), z_axis_from_pos(0), 1e-2);
  ASSERT_NEAR(z_axis(1), z_axis_from_pos(1), 1e-2);
  ASSERT_NEAR(z_axis(2), z_axis_from_pos(2), 1e-2);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "roi_to_plane_converter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
