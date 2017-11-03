#include <gtest/gtest.h>

#include "aerial_autonomy/trackers/closest_tracking_strategy.h"
#include "aerial_autonomy/trackers/id_tracking_strategy.h"
#include "aerial_autonomy/trackers/simple_multi_tracker.h"

TEST(SimpleMultiTrackerTests, Constructor) {
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy()));
}

TEST(SimpleMultiTrackerTests, SetTrackingVectors) {
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy()));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));
  simple_tracker.setTrackingVectors(tracking_vectors);

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors_get;
  simple_tracker.getTrackingVectors(tracking_vectors_get);

  ASSERT_EQ(tracking_vectors_get.size(), tracking_vectors.size());

  for (auto itr : tracking_vectors_get) {
    ASSERT_EQ(tracking_vectors_get[itr.first], tracking_vectors[itr.first]);
  }
}

TEST(ClosestTrackingStrategyTests, GetTrackingVectorClosest) {
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy()));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));
  simple_tracker.setTrackingVectors(tracking_vectors);

  simple_tracker.initialize();

  tf::Transform tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);
}

TEST(ClosestTrackingStrategyTests, ClosestTrackingLostNoRetries) {
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy(0)));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));
  simple_tracker.setTrackingVectors(tracking_vectors);

  simple_tracker.initialize();
  tf::Transform tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);

  // Remove tracked vector
  tracking_vectors.erase(0);
  simple_tracker.setTrackingVectors(tracking_vectors);
  ASSERT_FALSE(simple_tracker.getTrackingVector(tracking_vector));
}

TEST(ClosestTrackingStrategyTests, ClosestTrackingLostMultipleRetries) {
  int retries = 5;
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy(retries)));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));
  simple_tracker.setTrackingVectors(tracking_vectors);

  simple_tracker.initialize();
  tf::Transform tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);

  // Remove tracked vector
  auto old_vector = tracking_vectors[0];
  tracking_vectors.erase(0);
  simple_tracker.setTrackingVectors(tracking_vectors);

  // Test retries
  for (int i = 0; i < retries; i++) {
    ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
    ASSERT_EQ(tracking_vector, old_vector);
  }
  ASSERT_FALSE(simple_tracker.getTrackingVector(tracking_vector));
}

TEST(ClosestTrackingStrategyTests, ClosestTrackingLostMultipleRetriesReinit) {
  int retries = 5;
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new ClosestTrackingStrategy(retries)));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));
  simple_tracker.setTrackingVectors(tracking_vectors);

  simple_tracker.initialize();

  // Get vector
  tf::Transform tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);

  // Reset vector
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 2));
  simple_tracker.setTrackingVectors(tracking_vectors);
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);

  // Remove tracked vector
  auto old_vector = tracking_vectors[0];
  tracking_vectors.erase(0);
  simple_tracker.setTrackingVectors(tracking_vectors);
  // Test retries
  for (int i = 0; i < retries; i++) {
    ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
    ASSERT_EQ(tracking_vector, old_vector);
  }
  ASSERT_FALSE(simple_tracker.getTrackingVector(tracking_vector));

  // Reinitialize tracker
  simple_tracker.initialize();
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[1]);
}

TEST(IdTrackingStrategyTests, IdFound) {
  uint32_t id = 0;
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new IdTrackingStrategy(id)));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));
  simple_tracker.setTrackingVectors(tracking_vectors);

  // Get vector
  tf::Transform tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[id]);

  // Track a different id
  id = 1;
  simple_tracker.setTrackingStrategy(
      std::unique_ptr<TrackingStrategy>(new IdTrackingStrategy(id)));

  // Get vector
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[id]);
}

TEST(IdTrackingStrategyTests, IdNotFound) {
  uint32_t id = 5;
  SimpleMultiTracker simple_tracker(
      std::unique_ptr<TrackingStrategy>(new IdTrackingStrategy(id)));

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  tracking_vectors[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tracking_vectors[1] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 3));

  tf::Transform tracking_vector;
  ASSERT_FALSE(simple_tracker.getTrackingVector(tracking_vector));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
