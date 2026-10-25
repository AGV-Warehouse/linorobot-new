#include <gtest/gtest.h>
#include "linorobot2_fleet_bridge/runtime_policy.hpp"
using namespace fleet;
TEST(Localization, OldMatchingPoseCannotCompleteANewRequest) {
  LocalizationConfirmation c;c.begin(10000000000LL,{1,2,0});
  c.observe(9000000000LL,{1,2,0},.01,.01,.01,.12);
  EXPECT_FALSE(c.confirmed(11000000000LL,11000000000LL,2));
  c.observe(11000000000LL,{1,2,0},.01,.01,.01,.12);
  EXPECT_FALSE(c.confirmed(9000000000LL,11000000000LL,2));
  EXPECT_TRUE(c.confirmed(11000000000LL,11000000000LL,2));
  EXPECT_FALSE(c.confirmed(14000000000LL,14000000000LL,2));
}
TEST(Localization, UncertainOrWrongPoseDoesNotConfirm) {
  LocalizationConfirmation c;c.begin(100,{1,2,0});
  c.observe(200,{1,2,0},.9,.01,.01,.12);EXPECT_FALSE(c.confirmed(200,200,2));
  c.observe(200,{8,2,0},.01,.01,.01,.12);EXPECT_FALSE(c.confirmed(200,200,2));
  c.observe(200,{1,2,1},.01,.01,.01,.12);EXPECT_FALSE(c.confirmed(200,200,2));
  c.observe(200,{1,2,0},.01,.01,.01,.12);EXPECT_TRUE(c.confirmed(200,200,2));
  c.begin(300,{1,2,0});EXPECT_FALSE(c.confirmed(400,400,2));
}
