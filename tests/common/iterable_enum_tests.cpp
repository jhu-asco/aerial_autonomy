#include <gtest/gtest.h>

#include <aerial_autonomy/common/iterable_enum.h>

enum class TestEnum {
  Red,
  Orange,
  Black,
  Yellow,
  Blue,
  First = Red,
  Last = Blue
};

TEST(IterableEnumTests, Constructor) {
  ASSERT_NO_THROW(IterableEnum<TestEnum>());
}

TEST(IterableEnumTests, Begin) {
  IterableEnum<TestEnum> itr_enum;
  ASSERT_EQ(*begin(itr_enum), TestEnum::Red);
}

TEST(IterableEnumTests, End) {
  IterableEnum<TestEnum> itr_enum;
  ASSERT_EQ(((int)*end(itr_enum)), ((int)TestEnum::Blue) + 1);
}

TEST(IterableEnumTests, Increment) {
  IterableEnum<TestEnum> itr_enum;
  auto itr = begin(itr_enum);
  itr.operator++();
  ASSERT_EQ(*itr, TestEnum::Orange);
}

TEST(IterableEnumTests, Iterate) {
  std::vector<TestEnum> enum_vec = {TestEnum::Red, TestEnum::Orange,
                                    TestEnum::Black, TestEnum::Yellow,
                                    TestEnum::Blue};
  int i = 0;
  for (auto color : IterableEnum<TestEnum>()) {
    ASSERT_EQ(color, enum_vec[i]);
    i++;
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}