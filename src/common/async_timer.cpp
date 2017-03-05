#include <aerial_autonomy/common/async_timer.h>
#include <boost/chrono/thread_clock.hpp>

AsyncTimer::AsyncTimer(boost::function<void()> function,
                       boost::chrono::duration<double> timer_duration)
    : function_(function), timer_duration_(timer_duration), running_(false) {}

void AsyncTimer::start() {
  assert(!running_);
  running_ = true;
  boost::thread{boost::bind(&AsyncTimer::functionTimer, this)};
}

void AsyncTimer::functionTimer() {
  while (true) {
    boost::chrono::thread_clock::time_point start =
        boost::chrono::thread_clock::now();
    function_();
    boost::this_thread::sleep_until(start + timer_duration_);
  }
}
