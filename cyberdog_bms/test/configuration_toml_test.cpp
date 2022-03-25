#include <dirent.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <algorithm>
#include "gtest/gtest.h"


TEST(hello, hello__Test)
{
  std::cout << "hello, gtest!" << std::endl;
  EXPECT_EQ('a', 97);
}


int main(int argc, char ** argv)
{
  // std::cout << BenchmarkPath << std::endl;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
