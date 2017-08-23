#include <gtest/gtest.h>

#include <cstdio>
#include <thread>

#include "aerial_autonomy/log/data_stream.h"
#include "aerial_autonomy/tests/test_utils.h"

class DataStreamTest : public testing::Test {
public:
  DataStreamTest() : test_path_("/tmp/dstest") {
    std::remove(test_path_.c_str());
    config_.set_delimiter(",");
  }

protected:
  DataStreamConfig config_;
  std::string test_path_;
};

TEST_F(DataStreamTest, Constructor) {
  ASSERT_NO_THROW(DataStream(test_path_, config_));
}

TEST_F(DataStreamTest, ConstructorBadPath) {
  ASSERT_THROW(DataStream("", config_), std::runtime_error);
}

TEST_F(DataStreamTest, DoubleStartl) {
  DataStream ds(test_path_, config_);
  ASSERT_THROW(ds << DataStream::startl << DataStream::startl,
               std::logic_error);
}

TEST_F(DataStreamTest, WriteOneLine) {
  std::unique_ptr<DataStream> ds(new DataStream(test_path_, config_));
  std::vector<std::vector<int>> data = {{10, 8, 3, 2, -1}};
  *ds << DataStream::startl;
  for (auto i : data[0]) {
    *ds << i;
  }
  *ds << DataStream::endl;
  ds->write();

  test_utils::verifyFileData(data, ds->path(), config_.delimiter());
}

TEST_F(DataStreamTest, WriteMultipleLines) {
  std::unique_ptr<DataStream> ds(new DataStream(test_path_, config_));
  std::vector<std::vector<int>> data = {
      {10, 8, 3, 2, -1}, {-3, -4, 5, 6}, {4, 5, 6, 0}};
  for (auto line : data) {
    *ds << DataStream::startl;
    for (auto i : line) {
      *ds << i;
    }
    *ds << DataStream::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  ds->write();

  test_utils::verifyFileData(data, ds->path(), config_.delimiter());
}

TEST_F(DataStreamTest, StreamTooFast) {
  config_.set_log_rate(1);
  std::unique_ptr<DataStream> ds(new DataStream(test_path_, config_));
  std::vector<std::vector<int>> data = {{10, 8, 3, 2, -1}, {-3, -4, 5, 6}};
  for (auto line : data) {
    *ds << DataStream::startl;
    for (auto i : line) {
      *ds << i;
    }
    *ds << DataStream::endl;
  }
  ds->write();

  std::vector<std::vector<int>> first_point(data.begin(), data.begin() + 1);
  test_utils::verifyFileData(first_point, ds->path(), config_.delimiter());
}

TEST_F(DataStreamTest, WriteWhileStreaming) {
  config_.set_log_rate(1);
  std::unique_ptr<DataStream> ds(new DataStream(test_path_, config_));
  std::vector<std::vector<int>> data = {{10, 8, 3, 2, -1}, {-3, -4, 5, 6}};
  for (size_t j = 0; j < data.size(); j++) {
    *ds << DataStream::startl;
    for (auto i : data[j]) {
      *ds << i;
    }
    if (j == 0)
      *ds << DataStream::endl;
  }
  ds->write();

  std::vector<std::vector<int>> first_point(data.begin(), data.begin() + 1);
  test_utils::verifyFileData(first_point, ds->path(), config_.delimiter());
}

TEST_F(DataStreamTest, LogDataFalse) {
  config_.set_log_rate(10);
  config_.set_log_data(false);
  std::unique_ptr<DataStream> ds(new DataStream(test_path_, config_));
  std::vector<std::vector<int>> data = {{10, 8, 3, 2, -1}, {-3, -4, 5, 6}};
  for (size_t j = 0; j < data.size(); j++) {
    *ds << DataStream::startl;
    for (auto i : data[j]) {
      *ds << i;
    }
    *ds << DataStream::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  ds->write();

  test_utils::verifyFileData(std::vector<std::vector<int>>(), ds->path(),
                             config_.delimiter());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
