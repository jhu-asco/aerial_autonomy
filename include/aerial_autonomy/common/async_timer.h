#pragma once

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

class AsyncTimer {
public:
  AsyncTimer(boost::function<void()> function,
             boost::chrono::duration<double> timer_duration);
  void start();

private:
  void functionTimer();

  bool running_;
  boost::function<void()> function_;
  boost::chrono::duration<double> timer_duration_;
};
