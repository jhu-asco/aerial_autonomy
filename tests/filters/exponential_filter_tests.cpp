#include <gtest/gtest.h>

#include <aerial_autonomy/filters/exponential_filter.h>

TEST(ExponentialFilterTests, Constructor) {
  ASSERT_NO_THROW(ExponentialFilter<double>(0.2));
}

TEST(ExponentialFilterTests, Initialize) {
  ExponentialFilter<double> filter(0.2);
  ASSERT_THROW(filter.getFilterData(), std::runtime_error);
}

TEST(ExponentialFilterTests, GainTooHigh) {
  ASSERT_DEATH(ExponentialFilter<double>(2.0),
               "Exponential gain should be less than or equal to 1.0");
  ASSERT_DEATH(ExponentialFilter<double>(-1.0),
               "Exponential gain should be greater than or equal to 0.0");
}

TEST(ExponentialFilterTests, OneStep) {
  ExponentialFilter<double> filter(0.1);
  double sensor_data = 25.0;
  double filter_data = filter.addAndFilter(sensor_data);
  ASSERT_DOUBLE_EQ(filter_data, sensor_data);
}

TEST(ExponentialFilterTests, ZeroGain) {
  ExponentialFilter<double> filter(0.0);
  double init_sensor_data = 25.0;
  double filter_data = filter.addAndFilter(init_sensor_data);
  filter_data = filter.addAndFilter(30.0);
  ASSERT_DOUBLE_EQ(filter_data, init_sensor_data);
}

TEST(ExponentialFilterTests, FullGain) {
  ExponentialFilter<double> filter(1.0);
  double init_sensor_data = 25.0;
  double filter_data = filter.addAndFilter(init_sensor_data);
  filter_data = filter.addAndFilter(30.0);
  ASSERT_DOUBLE_EQ(filter_data, 30.0);
}

TEST(ExponentialFilterTests, Convergence) {
  ExponentialFilter<double> filter(0.1);
  double sensor_data = 25.0;
  filter.add(0.0); // Initial sensor
  for (int i = 0; i < 100; ++i) {
    filter.add(sensor_data);
  }
  ASSERT_NEAR(filter.getFilterData(), sensor_data, 1e-3);
}

TEST(ExponentialFilterTests, DataAvailable) {
  ExponentialFilter<double> filter(0.1);
  ASSERT_FALSE(filter.isDataAvailable());

  filter.add(0.0);
  ASSERT_TRUE(filter.isDataAvailable());

  filter.reset();
  ASSERT_FALSE(filter.isDataAvailable());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
