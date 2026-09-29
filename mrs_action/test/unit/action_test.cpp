#include <gtest/gtest.h>
#include "action_service_traits.hpp"

TEST(mrs_action, test_test){
    ASSERT_TRUE(true);
}
TEST(mrs_action, test_test_2){
    ASSERT_TRUE(true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}