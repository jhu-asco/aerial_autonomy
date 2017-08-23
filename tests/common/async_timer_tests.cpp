#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <aerial_autonomy/common/async_timer.h>

class AsyncTimerTests : public ::testing::Test {
public:
  void emptyFunction() {}
  void counterFunction() { x++; }
  int x = 0;
};

TEST_F(AsyncTimerTests, Constructor) {
  AsyncTimer timer(std::bind(&AsyncTimerTests::emptyFunction, this),
                   std::chrono::milliseconds(10));
}

TEST_F(AsyncTimerTests, Start) {
  AsyncTimer timer(std::bind(&AsyncTimerTests::counterFunction, this),
                   std::chrono::milliseconds(50));
  ASSERT_EQ(0, this->x);
  timer.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(1, this->x);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  ASSERT_EQ(2, this->x);
}

TEST_F(AsyncTimerTests, DoubleStart) {
  AsyncTimer timer(std::bind(&AsyncTimerTests::emptyFunction, this),
                   std::chrono::milliseconds(10));
  timer.start();
  ASSERT_THROW(timer.start(), std::logic_error);
}

TEST_F(AsyncTimerTests, Timing) {
  AsyncTimer timer(std::bind(&AsyncTimerTests::counterFunction, this),
                   std::chrono::milliseconds(20));
  ASSERT_EQ(0, this->x);
  timer.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_GE(this->x, 49);
  ASSERT_LE(this->x, 51);
  timer.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_GE(this->x, 49);
  ASSERT_LE(this->x, 51);
  timer.setDuration(std::chrono::milliseconds(40));
  timer.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_GE(this->x, 74);
  ASSERT_LE(this->x, 76);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
