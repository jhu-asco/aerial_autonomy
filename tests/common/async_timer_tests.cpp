#include <gtest/gtest.h>

#include <aerial_autonomy/common/async_timer.h>

class AsyncTimerTests : public ::testing::Test {
public:
  void emptyFunction() {}
  void counterFunction() { x++; }
  int x = 0;
};

TEST_F(AsyncTimerTests, Constructor) {
  AsyncTimer timer(boost::bind(&AsyncTimerTests::emptyFunction, this),
                   boost::chrono::seconds(1));
}

TEST_F(AsyncTimerTests, Start) {
  AsyncTimer timer(boost::bind(&AsyncTimerTests::counterFunction, this),
                   boost::chrono::milliseconds(50));
  ASSERT_EQ(0, this->x);
  timer.start();
  boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  ASSERT_EQ(1, this->x);
  boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  ASSERT_EQ(2, this->x);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
