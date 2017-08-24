#include <gtest/gtest.h>

#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <boost/filesystem.hpp>

class LogTest : public testing::Test {
public:
  LogTest() : test_path_("/tmp/log_test") {
    std::remove(test_path_.c_str());
    config_.set_directory(test_path_);
    config_.set_write_duration(500);
    for (int i = 0; i < 4; i++) {
      DataStreamConfig *ds = config_.add_data_stream_configs();
      ds->set_stream_id(std::string("stream") + std::to_string(i));
      ds->set_delimiter(",");
      ds->set_log_rate(50);
    }
  }

  template <typename T>
  void writeToStream(std::vector<std::vector<T>> log, std::string stream_id,
                     int log_duration) {
    for (auto line : log) {
      Log::instance()[stream_id] << DataStream::startl;
      for (auto i : line) {
        Log::instance()[stream_id] << i;
      }
      Log::instance()[stream_id] << DataStream::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(log_duration));
    }
  }

protected:
  LogConfig config_;
  std::string test_path_;
};

TEST_F(LogTest, Constructor) { ASSERT_NO_THROW(Log::instance()); }

TEST_F(LogTest, OneInstance) { ASSERT_EQ(&Log::instance(), &Log::instance()); }

TEST_F(LogTest, Configure) {
  ASSERT_NO_THROW(Log::instance().configure(config_));
  ASSERT_TRUE(boost::filesystem::exists(config_.directory()));
}

TEST_F(LogTest, ConfigureBadPath) {
  config_.set_directory("/");
  ASSERT_THROW(Log::instance().configure(config_), std::runtime_error);
}

TEST_F(LogTest, ConfigureStreamNotUnique) {
  DataStreamConfig *ds = config_.add_data_stream_configs();
  ds->set_stream_id("stream0");
  ASSERT_THROW(Log::instance().configure(config_), std::runtime_error);
}

TEST_F(LogTest, IndexOperator) {
  ASSERT_NO_THROW(Log::instance().configure(config_));
  ASSERT_FALSE(Log::instance()["bad_name"].configuration().log_data());

  DataStream &ds0 = Log::instance()["stream0"];
  ASSERT_EQ(ds0.configuration().stream_id(), std::string("stream0"));
  DataStream &ds1 = Log::instance()["stream1"];
  ASSERT_EQ(ds1.configuration().stream_id(), std::string("stream1"));
}

TEST_F(LogTest, AddDataStream) {
  ASSERT_NO_THROW(Log::instance().configure(config_));
  DataStreamConfig ds_config;

  ds_config.set_stream_id("new_stream");
  Log::instance().addDataStream(ds_config);

  DataStream &ds0 = Log::instance()[ds_config.stream_id()];
  ASSERT_EQ(ds0.configuration().stream_id(), ds_config.stream_id());
}

TEST_F(LogTest, Write) {
  ASSERT_NO_THROW(Log::instance().configure(config_));
  std::vector<std::vector<double>> data0 = {
      {8.8, -2.2, 3.5}, {5.6, -90.1, 4}, {3, 4, 5}};
  std::vector<std::vector<double>> data1 = {
      {6.8, -2.5, -1.5}, {5.2, -9.1, 4.1}, {3, 4.8, -5}};

  writeToStream(data0, "stream0", 30);
  writeToStream(data1, "stream1", 30);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  test_utils::verifyFileData(
      data0, Log::instance()["stream0"].path(),
      Log::instance()["stream0"].configuration().delimiter());
  test_utils::verifyFileData(
      data1, Log::instance()["stream1"].path(),
      Log::instance()["stream1"].configuration().delimiter());
}

TEST_F(LogTest, WriteSlowRate) {
  config_.set_write_duration(1000);
  ASSERT_NO_THROW(Log::instance().configure(config_));
  std::vector<std::vector<double>> data0 = {
      {8.8, -2.2, 3.5}, {5.6, -90.1, 4}, {3, 4, 5}};
  std::vector<std::vector<double>> data1 = {
      {6.8, -2.5, -1.5}, {5.2, -9.1, 4.1}, {3, 4.8, -5}};

  writeToStream(data0, "stream0", 30);
  writeToStream(data1, "stream1", 30);

  // Check for no write
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  test_utils::verifyFileData(
      std::vector<std::vector<double>>(), Log::instance()["stream0"].path(),
      Log::instance()["stream0"].configuration().delimiter());
  test_utils::verifyFileData(
      std::vector<std::vector<double>>(), Log::instance()["stream1"].path(),
      Log::instance()["stream1"].configuration().delimiter());
  // Wait for write
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  test_utils::verifyFileData(
      data0, Log::instance()["stream0"].path(),
      Log::instance()["stream0"].configuration().delimiter());
  test_utils::verifyFileData(
      data1, Log::instance()["stream1"].path(),
      Log::instance()["stream1"].configuration().delimiter());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
