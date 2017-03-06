#pragma once

#include <chrono>
#include <functional>
#include <thread>

class AsyncTimer {
public:
  AsyncTimer(std::function<void()> function,
             std::chrono::duration<double> timer_duration);
  virtual ~AsyncTimer();
  void start();

private:
  void functionTimer();

  std::thread timer_thread_;
  std::function<void()> function_;
  std::chrono::duration<double> timer_duration_;
  bool running_;
};
