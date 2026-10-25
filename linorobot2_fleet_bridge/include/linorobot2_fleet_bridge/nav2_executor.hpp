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
  bool busy() const {return busy_ || pending_goals_ || pending_cancels_ || cancellation_failed_;}
  bool executing() const {return following_;}
private:
  using Plan=nav2_msgs::action::ComputePathToPose;
  using Follow=nav2_msgs::action::FollowPath;
  rclcpp::Node& node_; RouteGuard& guard_;
  rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Client<Follow>::SharedPtr follower_;
  rclcpp_action::ClientGoalHandle<Plan>::SharedPtr plan_handle_;
  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr follow_handle_;
  std::function<void(bool)> done_;
  bool busy_=false;
  bool following_=false;
  unsigned generation_=0;
  unsigned pending_goals_=0, pending_cancels_=0;
  bool cancellation_failed_=false;
  template<class Client,class Handle> void cancel_handle(Client client,Handle handle) {
    if(!handle) return;
    ++pending_cancels_;
    client->async_cancel_goal(handle,[this](auto response){
      --pending_cancels_;
      // ERROR_NONE or ERROR_GOAL_TERMINATED. Other responses require process/operator recovery.
      if(response->return_code!=0 && response->return_code!=3) cancellation_failed_=true;
    });
  }
  void finish(bool result,unsigned generation);
};
}
