#include "linorobot2_fleet_bridge/nav2_executor.hpp"
namespace fleet {
Nav2Executor::Nav2Executor(rclcpp::Node& node,RouteGuard& guard):node_(node),guard_(guard) {
  planner_=rclcpp_action::create_client<Plan>(&node_,"compute_path_to_pose");
  follower_=rclcpp_action::create_client<Follow>(&node_,"follow_path");
}
void Nav2Executor::finish(bool result,unsigned generation) {
  if(generation!=generation_ || !busy_) return;
  busy_=false; following_=false; plan_handle_.reset();follow_handle_.reset();guard_.revoke(); auto callback=std::move(done_); if(callback) callback(result);
}
bool Nav2Executor::start(Pose target,std::function<void(bool)> done) {
  if(busy() || !guard_.active() || !planner_->action_server_is_ready() || !follower_->action_server_is_ready()) return false;
  busy_=true; done_=std::move(done); const auto generation=++generation_;
  Plan::Goal goal; goal.goal.header.frame_id="map"; goal.goal.header.stamp=node_.now();
  goal.goal.pose.position.x=target.x; goal.goal.pose.position.y=target.y;
  goal.goal.pose.orientation.z=std::sin(target.yaw/2); goal.goal.pose.orientation.w=std::cos(target.yaw/2);
  goal.planner_id="GridBased"; goal.use_start=false;
  auto options=rclcpp_action::Client<Plan>::SendGoalOptions();
  options.goal_response_callback=[this,generation](auto handle) {
    --pending_goals_;
    if(handle && generation!=generation_) cancel_handle(planner_,handle);
    else if(!handle) finish(false,generation);
    else plan_handle_=handle;
  };
  options.result_callback=[this,generation](const auto& result) {
    if(generation!=generation_) return;
    plan_handle_.reset();
    if(result.code!=rclcpp_action::ResultCode::SUCCEEDED) {finish(false,generation);return;}
    std::vector<Pose> path;
    for(const auto& p:result.result->path.poses) path.push_back({p.pose.position.x,p.pose.position.y,0});
    if(!guard_.accepts(path)) {finish(false,generation);return;}
    Follow::Goal follow; follow.path=result.result->path; follow.controller_id="FollowPath"; follow.goal_checker_id="general_goal_checker";
    auto opts=rclcpp_action::Client<Follow>::SendGoalOptions();
    opts.goal_response_callback=[this,generation](auto handle) {
      --pending_goals_;
      if(handle && generation!=generation_) cancel_handle(follower_,handle);
      else if(!handle) finish(false,generation);
      else {follow_handle_=handle;following_=true;}
    };
    opts.result_callback=[this,generation](const auto& r) {finish(r.code==rclcpp_action::ResultCode::SUCCEEDED,generation);};
    ++pending_goals_;follower_->async_send_goal(follow,opts);
  };
  ++pending_goals_;planner_->async_send_goal(goal,options); return true;
}
void Nav2Executor::cancel() {
  if(busy_) ++generation_;
  busy_=false;following_=false;done_=nullptr;guard_.revoke();
  // Cancel only owned goal IDs: a delayed cancel-all could kill a later resumed goal.
  cancel_handle(planner_,plan_handle_);plan_handle_.reset();
  cancel_handle(follower_,follow_handle_);follow_handle_.reset();
}
}
