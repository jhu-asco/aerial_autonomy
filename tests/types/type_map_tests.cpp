#include <gtest/gtest.h>
#include <aerial_autonomy/types/type_map.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct BaseClass {
  int i = 0;
};
struct SubClass1 : BaseClass {
  int i = 1;
};
struct SubClass2 : BaseClass {
  int i = 2;
};
///

/// \brief TEST
/// All the tests are defined here
TEST(TypeMapTests, SaveAndRetrieveObject) {
  TypeMap<BaseClass> type_map;
  SubClass1 subclass1;
  SubClass2 subclass2;
  type_map.addObject(subclass1);
  type_map.addObject(subclass2);
  ASSERT_NO_THROW(type_map.getObject<SubClass1>());
  ASSERT_NO_THROW(type_map.getObject<SubClass2>());
  SubClass1 *object1 = type_map.getObject<SubClass1>();
  SubClass2 *object2 = type_map.getObject<SubClass2>();
  ASSERT_EQ(object1->i, subclass1.i);
  ASSERT_EQ(object2->i, subclass2.i);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
