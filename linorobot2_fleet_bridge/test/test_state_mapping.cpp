#include <gtest/gtest.h>
#include <limits>
#include "linorobot2_fleet_bridge/robot_state.hpp"
using namespace fleet;
TEST(State, MissingAndStaleSensorsBlockMotion) {
  RobotState s;EXPECT_FALSE(s.ready(2));s.localized=true;s.safety_ok=true;s.battery=70;
  s.pose_at=s.velocity_at=s.safety_at=s.battery_at=RobotState::Clock::now();EXPECT_TRUE(s.ready(2));
  s.safety_at-=std::chrono::seconds(3);EXPECT_FALSE(s.ready(2));
}
TEST(State, NonfiniteVelocityBlocksMotion) {
  RobotState s;s.localized=true;s.safety_ok=true;s.battery=70;
  s.pose_at=s.velocity_at=s.safety_at=s.battery_at=RobotState::Clock::now();
  s.vx=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(s.ready(2));
}
