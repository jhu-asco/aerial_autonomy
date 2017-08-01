#include <gtest/gtest.h>

#include "aerial_autonomy/trackers/simple_multi_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

TEST(SimpleMultiTrackerTests, Constructor) {
  SimpleMultiTracker simple_tracker(new ClosestTrackingStrategy());
}

TEST(SimpleMultiTrackerTests, SetTrackingVectors) {
  SimpleMultiTracker simple_tracker(new ClosestTrackingStrategy());

  std::unordered_map<uint32_t, Position> tracking_vectors;
  tracking_vectors[0] = Position(1, 1, 1);
  tracking_vectors[1] = Position(1, 1, 3);
  simple_tracker.setTrackingVectors(tracking_vectors);

  std::unordered_map<uint32_t, Position> tracking_vectors_get;
  simple_tracker.getTrackingVectors(tracking_vectors_get);

  ASSERT_EQ(tracking_vectors_get.size(), tracking_vectors.size());

  for (auto itr : tracking_vectors_get) {
    ASSERT_EQ(tracking_vectors_get[itr.first], tracking_vectors[itr.first]);
  }
}

TEST(SimpleMultiTrackerTests, GetTrackingVectorClosest) {
  SimpleMultiTracker simple_tracker(new ClosestTrackingStrategy());

  std::unordered_map<uint32_t, Position> tracking_vectors;
  tracking_vectors[0] = Position(1, 1, 1);
  tracking_vectors[1] = Position(1, 1, 3);
  simple_tracker.setTrackingVectors(tracking_vectors);

  simple_tracker.initialize();

  Position tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);
}

TEST(SimpleMultiTrackerTests, ClosestTrackingLost) {
  SimpleMultiTracker simple_tracker(new ClosestTrackingStrategy());

  std::unordered_map<uint32_t, Position> tracking_vectors;
  tracking_vectors[0] = Position(1, 1, 1);
  tracking_vectors[1] = Position(1, 1, 3);
  simple_tracker.setTrackingVectors(tracking_vectors);

  simple_tracker.initialize();
  Position tracking_vector;
  ASSERT_TRUE(simple_tracker.getTrackingVector(tracking_vector));
  ASSERT_EQ(tracking_vector, tracking_vectors[0]);

  // Remove tracked vector
  tracking_vectors.erase(0);
  simple_tracker.setTrackingVectors(tracking_vectors);
  ASSERT_FALSE(simple_tracker.getTrackingVector(tracking_vector));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
