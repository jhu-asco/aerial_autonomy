#pragma once

#include "data_stream_config.pb.h"

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <chrono>
#include <fstream>

/**
 * @brief DataStream is a rate-limited output stream for data logging.
 * Users need to call its write() function to write its internal buffer to a
 * file.
 * This call will typically be done in a separate thread to ensure that logging
 * does not interfere with other tasks.
 */
class DataStream {
public:
  /**
  * @brief Constructor
  * @param path File path to write to
  * @param config Data stream configuration
  */
  DataStream(boost::filesystem::path path, DataStreamConfig config);

  /**
  * @brief Move operator
  * @param o Moving DataStream
  */
  DataStream(DataStream &&o);

  /**
  * @brief Write the internal buffer to the file stream
  */
  void write();

  /**
  * @brief Getter for config
  * @return Configuration
  */
  const DataStreamConfig &configuration();

  /**
  * @brief Get log path
  * @return The path
  */
  boost::filesystem::path path();

  /**
  * @brief Stream operator for stream modifiers
  * @param func Function pointer for modifying the stream
  * @return Modified DataStream
  */
  DataStream &operator<<(DataStream &(*func)(DataStream &));

  /**
  * @brief Streaming operator which writes data to a data point
  * @tparam T Class of variable getting written to data point
  * @param t Data to write to data point
  * @return modified DataStream
  */
  template <class T> DataStream &operator<<(const T &t) {
    if (config_.log_data() && streaming_) {
      data_point_ << config_.delimiter() << t;
    }
    return *this;
  }

  /**
   * @brief specialized operator to handle eigen vector
   * @param t Eigen Vector
   * @return modified DataStream
   */
  template <int row, int col>
  DataStream &operator<<(const Eigen::Matrix<double, row, col> &t) {
    if (config_.log_data() && streaming_) {
      for (int i = 0; i < t.size(); ++i) {
        data_point_ << config_.delimiter() << t(i);
      }
    }
    return *this;
  }

  /**
   * @brief DataStream modifier which signals the start of a data point
   * @param ds DataStream to modify
   * @return Modified DataStream
   */
  static DataStream &startl(DataStream &ds);
  /**
   * @brief DataStream modifier which signals the end of a data point
   * @param ds DataStream to modify
   * @return Modified DataStream
   */
  static DataStream &endl(DataStream &ds);
  /**
   * @brief DataStream modifier which signals the start of a header
   * @param ds DataStream to modify
   * @return Modified DataStream
   */
  static DataStream &starth(DataStream &ds);

private:
  /**
  * @brief Reset a string stream
  * @param ss String stream to reset
  */
  static void resetStringstream(std::stringstream &ss);

  DataStreamConfig config_;      ///< Configuration
  boost::filesystem::path path_; ///< Data filepath
  std::chrono::time_point<std::chrono::high_resolution_clock>
      last_write_time_; ///< Last time the buffer has been written to
  std::fstream fs_;     ///< File stream that is written to
  bool streaming_;      ///< Whether data is currently being recorded or not
  std::stringstream buffer_;     ///< Stores several data points until they are
                                 /// written to file stream
  std::stringstream data_point_; ///< Stores the current data point while it is
                                 /// written to the DataStream (i.e. while
                                 /// streaming_ == true)
  mutable boost::mutex buffer_mutex_; ///< Synchronize access to the buffer
};
