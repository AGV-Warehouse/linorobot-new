#include "linorobot2_fleet_bridge/robot_state.hpp"
namespace fleet {
bool RobotState::ready(double age) const {
  auto fresh=[age](auto at) {return std::chrono::duration<double>(Clock::now()-at).count()<=age;};
  return localized && safety_ok && battery && std::isfinite(*battery) && *battery>=0 && *battery<=100 &&
    fresh(pose_at) && fresh(velocity_at) && fresh(safety_at) && fresh(battery_at) &&
    std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw) &&
    std::isfinite(vx) && std::isfinite(wz);
}
}
