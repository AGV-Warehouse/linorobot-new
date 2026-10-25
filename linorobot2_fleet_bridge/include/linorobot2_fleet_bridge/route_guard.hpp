#pragma once
#include <cmath>
#include <vector>

namespace fleet {
struct Pose { double x=0, y=0, yaw=0; };
// Centreline corridor. Width is for robot CENTRES after footprint/braking margin.
class RouteGuard {
public:
  explicit RouteGuard(double half_width): width_(half_width) {}
  void authorize(Pose start, Pose end);
  void revoke() { active_=false; }
  bool contains(Pose point) const;
  bool accepts(const std::vector<Pose>& path) const;
  bool active() const { return active_; }
private:
  Pose start_, end_;
  double width_;
  bool active_=false;
};
}
