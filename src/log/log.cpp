#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/common/string_utils.h"

#include <boost/filesystem.hpp>

void Log::configure(LogConfig config) {
  config_ = config;

  directory_ = config_.directory() + string_utils::currentDateTimeString();
  if (!boost::filesystem::exists(directory_)) {
    if (!boost::filesystem::create_directory(directory_)) {
      throw std::runtime_error("Could not create Log directory: " +
                               directory_.string());
    }
  }
  configureStreams(config_);
}

boost::filesystem::path Log::directory() { return directory_; }

DataStream &Log::operator[](std::string id) {
  auto stream = streams_.find(id);
  if (stream == streams_.end()) {
    throw std::runtime_error("DataStream with id \"" + id +
                             "\" does not exist");
  }
  return stream->second;
}

void Log::addDataStream(DataStreamConfig stream_config) {
  if (streams_.find(stream_config.stream_id()) != streams_.end()) {
    throw std::runtime_error("Stream ID not unique: " +
                             stream_config.stream_id());
  }
  streams_.emplace(
      stream_config.stream_id(),
      DataStream(directory_ / stream_config.stream_id(), stream_config));
}

void Log::configureStreams(LogConfig config) {
  log_timer_.stop();
  streams_.clear(); // streams are closed in destructor
  for (auto stream_config : config_.data_stream_configs()) {
    addDataStream(stream_config);
  }
  log_timer_.setDuration(std::chrono::milliseconds(config_.write_duration()));
  log_timer_.start();
}

void Log::writeStreams() {
  for (auto &stream : instance().streams_) {
    stream.second.write();
  }
}
