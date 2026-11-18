#include <gtest/gtest.h>
#include "linorobot2_fleet_bridge/runtime_policy.hpp"
using namespace fleet;
TEST(Heading, UnconstrainedTransitFollowsIncomingEdgeButParkingKeepsExplicitHeading) {
  const auto west=navigation_target({7,2,0},{4,2,0},false);
  EXPECT_DOUBLE_EQ(west.x,4); EXPECT_DOUBLE_EQ(west.y,2);
  EXPECT_NEAR(std::abs(west.yaw),std::acos(-1.0),1e-12);
  EXPECT_NEAR(navigation_target({4,2,0},{4,5,0},false).yaw,std::acos(-1.0)/2,1e-12);
  EXPECT_DOUBLE_EQ(navigation_target({7,2,0},{4,2,0},true).yaw,0);
  EXPECT_DOUBLE_EQ(navigation_target({4,2,0},{4,2,.5},false).yaw,.5);
}
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

TEST(Localization, NomotionRequestDoesNotMakePreRequestScanFresh) {
  LocalizationConfirmation c;c.begin(62908000000LL,{-4,0,0});
  // The initial-pose response can arrive later while retaining an older scan stamp.
  c.observe(62900000000LL,{-4.0015,.0005,0},.01,.01,.01,.12);
  EXPECT_FALSE(c.confirmed(63100000000LL,63100000000LL,2));
  // A real subsequent AMCL estimate and TF must both advance after the request.
  c.observe(63200000000LL,{-4.0015,.0005,0},.01,.01,.01,.12);
  EXPECT_FALSE(c.confirmed(63100000000LL,63300000000LL,2));
  EXPECT_TRUE(c.confirmed(63300000000LL,63300000000LL,2));
}

TEST(Heading, KernelThetaRangeIncludesFieldRegressionAndPiBoundaries) {
  const double pi=std::acos(-1.0);
  for(const double angle:{3.1517887273207705,pi,-pi,pi+1e-9,-pi-1e-9}) {
    const auto wrapped=normalized_yaw(angle);
    EXPECT_GE(wrapped,-pi);EXPECT_LE(wrapped,pi);
    EXPECT_NEAR(normalized_yaw(wrapped-angle),0,1e-12);
    const double z=std::sin(angle/2),w=std::cos(angle/2);
    const auto yaw=quaternion_yaw(0,0,z,w);
    EXPECT_GE(yaw,-pi);EXPECT_LE(yaw,pi);
    EXPECT_NEAR(normalized_yaw(yaw-wrapped),0,1e-12);
    EXPECT_DOUBLE_EQ(yaw,quaternion_yaw(0,0,-z,-w));
  }
  EXPECT_LT(normalized_yaw(3.1517887273207705),0);
}
TEST(Localization, QuaternionSignAndWrappedPiPoseConfirmSameHeading) {
  const double yaw=3.1517887273207705,z=std::sin(yaw/2),w=std::cos(yaw/2);
  LocalizationConfirmation c;c.begin(100,{-4,0,normalized_yaw(yaw)});
  c.observe(200,{-4,0,quaternion_yaw(0,0,-z,-w)},.01,.01,.01,.12);
  EXPECT_TRUE(c.confirmed(200,200,2));
}
