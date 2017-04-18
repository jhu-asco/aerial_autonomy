#include <gtest/gtest.h>

#include <thread>

#include "aerial_autonomy/common/atomic.h"

class AtomicThreadTest : public ::testing::Test {
public:
  AtomicThreadTest() : a(5) {}
  void increment() { a.set(a.get() + 1); }

private:
  Atomic<int> a;
};

TEST(AtomicTests, DefaultConstructor) { Atomic<int> a; }

TEST(AtomicTests, Constructor) {
  Atomic<int> a(5);
  ASSERT_EQ(a.get(), 5);
}

TEST(AtomicTests, CopyConstructor) {
  Atomic<int> a(5);
  Atomic<int> b(a);
  ASSERT_EQ(b.get(), 5);
}

TEST(AtomicTests, SetGet) {
  Atomic<int> a(6);
  a.set(2);
  ASSERT_EQ(a.get(), 2);
  a.set(10);
  ASSERT_EQ(a.get(), 10);
}

TEST(AtomicTests, TAssignment) {
  Atomic<int> a;
  a = 5;
  ASSERT_EQ(a.get(), 5);
  a = -10;
  ASSERT_EQ(a.get(), -10);
}

TEST(AtomicTests, AtomicAssignment) {
  Atomic<int> a(5);
  Atomic<int> b(1);
  b = a;
  ASSERT_EQ(b.get(), 5);
}

TEST(AtomicTests, T) {
  Atomic<int> a(2);
  int i = a;
  ASSERT_EQ(i, 2);
}

TEST_F(AtomicThreadTest, ThreadSafe) {
  // Test for race condition seg fault
  std::thread t1(std::bind(&AtomicThreadTest::increment, this));
  std::thread t2(std::bind(&AtomicThreadTest::increment, this));
  auto start = std::chrono::high_resolution_clock::now();
  std::this_thread::sleep_until(start + std::chrono::duration<double>(0.1));
  t1.join();
  t2.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
