#include "aerial_autonomy/log/data_stream.h"

#include <exception>

DataStream::DataStream(std::string path, DataStreamConfig config)
    : config_(config), path_(path), streaming_(false) {
  fs_.open(path, std::fstream::out);
  if (!fs_.is_open()) {
    throw std::runtime_error("Could not open file: " + path);
  }
}

DataStream::DataStream(DataStream &&o) {
  config_ = o.config_;
  last_write_time_ = o.last_write_time_;
  path_ = o.path_;
  o.fs_.close();

  fs_.open(path_, std::fstream::out);
  if (!fs_.is_open()) {
    throw std::runtime_error("Could not open file: " + path_);
  }
  streaming_ = false;
}

void DataStream::write() {
  boost::mutex::scoped_lock lock(buffer_mutex_);

  fs_ << buffer_.str();
  buffer_.str(std::string());
  buffer_.clear();
}

DataStream &DataStream::operator<<(DataStream &(*func)(DataStream &)) {
  return func(*this);
}

DataStream &DataStream::startl(DataStream &ds) {
  if (ds.streaming_) {
    throw std::logic_error("startl called on streaming DataStream");
  }
  auto now = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_diff = now - ds.last_write_time_;
  ds.streaming_ = time_diff.count() > 1. / ds.config_.log_rate();
  if (ds.streaming_) {
    ds.data_point_ << now.time_since_epoch().count();
  }
  return ds;
}

DataStream &DataStream::endl(DataStream &ds) {
  if (ds.streaming_) {
    {
      boost::mutex::scoped_lock(buffer_mutex_);
      ds.buffer_ << ds.data_point_.str() << std::endl;
    }
    ds.last_write_time_ = std::chrono::high_resolution_clock::now();
    ds.data_point_.clear();
    ds.data_point_.str(std::string());
    ds.streaming_ = false;
  }
  return ds;
}
