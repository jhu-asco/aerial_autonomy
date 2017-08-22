#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

/**
 * @brief Calls given function on a timer in its own thread
 */
class AsyncTimer {
public:
  /**
   * @brief Constructor
   * @param function Function to call
   * @param timer_duration The amount of time in between each function call
   */
  AsyncTimer(std::function<void()> function,
             std::chrono::duration<double> timer_duration);
  /**
   * @brief Destructor cleans up running thread
   */
  virtual ~AsyncTimer();

  /**
   * @brief Starts running the timer thread
   */
  void start();

  /**
   * @brief Stops running the timer thread
   */
  void stop();

  /**
   * @brief Set the duration of the timer
   * @param duration The duration to set
   */
  void setDuration(std::chrono::duration<double> duration);

private:
  /**
   * @brief The timer loop
   */
  void functionTimer();

  std::thread timer_thread_;       ///< The thread used for the timer
  std::function<void()> function_; ///< The function called by the timer
  std::chrono::duration<double>
      timer_duration_; ///< The amount of time in between each function call
  std::atomic_bool running_; ///< True when the timer is running
};
