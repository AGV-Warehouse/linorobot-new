#pragma once
#include "linorobot2_fleet_bridge/route_guard.hpp"
#include <cmath>
#include <cstdint>
namespace fleet {
// A matching old TF is not localization confirmation. Both AMCL and TF must
// advance past the request; AMCL must also report bounded uncertainty.
class LocalizationConfirmation {
  std::int64_t requested_=0, evidence_=0;
  Pose target_{};
public:
  void begin(std::int64_t requested, Pose target) {requested_=requested;target_=target;evidence_=0;}
  void observe(std::int64_t stamp, Pose pose, double variance_x, double variance_y,
               double variance_yaw, double tolerance) {
    const double yaw=std::atan2(std::sin(pose.yaw-target_.yaw),std::cos(pose.yaw-target_.yaw));
    if(stamp>requested_ && std::isfinite(variance_x) && std::isfinite(variance_y) && std::isfinite(variance_yaw) &&
       variance_x>=0 && variance_x<=0.25 && variance_y>=0 && variance_y<=0.25 &&
       variance_yaw>=0 && variance_yaw<=0.0685 &&
       std::hypot(pose.x-target_.x,pose.y-target_.y)<=tolerance && std::abs(yaw)<0.15)
      evidence_=stamp;
  }
  bool confirmed(std::int64_t transform_stamp, std::int64_t now, double max_age) const {
    return evidence_>requested_ && transform_stamp>=evidence_ && transform_stamp>requested_ &&
      now>=evidence_ && (now-evidence_)*1e-9<=max_age;
  }
};
}
