#pragma once
#include "linorobot2_fleet_bridge/route_guard.hpp"
#include <chrono>
#include <optional>
#include <string>
namespace fleet {
struct RobotState {
  using Clock=std::chrono::steady_clock;
  Pose pose;
  double vx=0, wz=0;
  std::optional<double> battery;
  bool safety_ok=false, charging=false;
  std::string e_stop;
  bool field_violation=true;
  Clock::time_point pose_at{}, velocity_at{}, battery_at{}, safety_at{};
  bool localized=false;
  bool ready(double max_age) const;
  bool stopped() const { return std::abs(vx)<0.005 && std::abs(wz)<0.01; }
};
}
