#include "linorobot2_fleet_bridge/route_guard.hpp"
#include <algorithm>
#include <stdexcept>
namespace fleet {
void RouteGuard::authorize(Pose start, Pose end) {
  if (!std::isfinite(width_) || width_<=0 || !std::isfinite(start.x) ||
      !std::isfinite(start.y) || !std::isfinite(end.x) || !std::isfinite(end.y))
    throw std::invalid_argument("Invalid route geometry");
  start_=start; end_=end; active_=true;
}
bool RouteGuard::contains(Pose p) const {
  if (!active_ || !std::isfinite(p.x) || !std::isfinite(p.y)) return false;
  const auto dx=end_.x-start_.x, dy=end_.y-start_.y, length2=dx*dx+dy*dy;
  const auto t=length2>1e-12 ? std::clamp(((p.x-start_.x)*dx+(p.y-start_.y)*dy)/length2,0.0,1.0) : 0.0;
  return std::hypot(p.x-start_.x-t*dx,p.y-start_.y-t*dy)<=width_;
}
bool RouteGuard::accepts(const std::vector<Pose>& path) const {
  if (path.empty()) return false;
  for (auto p:path) if (!contains(p)) return false;
  // The capsule is convex, so the straight segments joining accepted points stay inside it.
  return true;
}
}
