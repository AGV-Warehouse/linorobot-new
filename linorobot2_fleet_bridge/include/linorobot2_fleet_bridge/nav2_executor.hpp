#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include "linorobot2_fleet_bridge/route_guard.hpp"
#include <functional>
namespace fleet {
class Nav2Executor {
public:
  explicit Nav2Executor(rclcpp::Node& node,RouteGuard& guard);
  bool start(Pose target,std::function<void(bool)> done);
  void cancel();
  bool busy() const {return busy_;}
  bool executing() const {return following_;}
private:
  using Plan=nav2_msgs::action::ComputePathToPose;
  using Follow=nav2_msgs::action::FollowPath;
  rclcpp::Node& node_; RouteGuard& guard_;
  rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Client<Follow>::SharedPtr follower_;
  std::function<void(bool)> done_;
  bool busy_=false;
  bool following_=false;
  unsigned generation_=0;
  void finish(bool result,unsigned generation);
};
}
