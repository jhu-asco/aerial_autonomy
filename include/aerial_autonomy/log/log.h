#pragma once

#include "aerial_autonomy/common/async_timer.h"
#include "aerial_autonomy/log/data_stream.h"

#include "log_config.pb.h"

#include <chrono>
#include <unordered_map>

#include <boost/thread/recursive_mutex.hpp>

/**
 * @brief Helper function to access a log stream and push a new start line
 */
#define DATA_LOG(stream) (Log::instance()[stream] << DataStream::startl)
/**
 * @brief Helper function to access a log stream and push a new start header
 */
#define DATA_HEADER(stream) (Log::instance()[stream] << DataStream::starth)

/**
 * @brief Manages data log streams and a timer for periodically writing streams
 * to file
 */
class Log {

public:
  /**
  * @brief Returns the only Log instance
  * @return the Singleton instance
  */
  static Log &instance() {
    // The only instance
    // Guaranteed to be lazy initialized
    // Guaranteed that it will be destroyed correctly
    static Log instance;
    return instance;
  }

  /**
  * @brief Destructor
  */
  ~Log();

  /**
  * @brief Configure the Log instance
  * @param config The configuration
  */
  void configure(LogConfig config);

  /**
  * @brief Index operator for retrieving a data stream
  * @param id ID of DataStream to get
  * @return DataStream with ID id
  */
  DataStream &operator[](std::string id);

  /**
  * @brief Add a data stream to the log
  * @param stream_config Configuration of the stream to add
  */
  void addDataStream(DataStreamConfig stream_config);

  /**
  * @brief Get the log directory
  * @return The path to the log directory
  */
  boost::filesystem::path directory();

  /**
   * @brief Delete the copy constructor
   *
   * @param The RHS Log to copy
   */
  Log(Log const &) = delete;
  /**
   * @brief Delete assign operator
   *
   * @param RHS Log to assign
   */
  void operator=(Log const &) = delete;

private:
  /**
  * @brief Setup streams and start write timer
  * @param config Log configuration
  */
  void configureStreams(LogConfig config);

  /**
  * @brief Write stream buffers to file.
  */
  void writeStreams();

  /**
   * @brief Constructor for creating log
   */
  Log()
      : config_(),
        log_timer_(std::bind(&Log::writeStreams, std::ref(*this)),
                   std::chrono::milliseconds(config_.write_duration())) {}

  /**
   * @brief Config specifying streams and frequencies etc
   */
  LogConfig config_;
  /**
   * @brief Map storing the stream key and Datastream class
   */
  std::unordered_map<std::string, DataStream> streams_;
  /**
   * @brief Timer to write log data
   */
  AsyncTimer log_timer_;
  /**
   * @brief Log folder where logs are stored
   */
  boost::filesystem::path directory_;
  /**
   * @brief Ensure creation/access/configure/write all
   * are synced even called from multiple threads
   */
  boost::recursive_mutex streams_mutex;
};
