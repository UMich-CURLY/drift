#include <gtest/gtest.h>

TEST(MathTest, PlusTest) {
  int a = 1;
  int b = 2;
  int c = a + b;
  EXPECT_EQ(c, 3);
}