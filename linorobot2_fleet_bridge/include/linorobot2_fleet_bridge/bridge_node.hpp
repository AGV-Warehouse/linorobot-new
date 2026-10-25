#pragma once
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vda5050_core/execution/protocol_adapter.hpp>
#include "linorobot2_fleet_bridge/nav2_executor.hpp"
#include "linorobot2_fleet_bridge/robot_state.hpp"
#include "linorobot2_fleet_bridge/vda_actions.hpp"
namespace fleet {
class BridgeNode: public rclcpp::Node {
public:
  BridgeNode();
  ~BridgeNode() override;
private:
  std::unique_ptr<Journal> journal_;
  std::unique_ptr<OrderBook> book_;
  std::unique_ptr<RouteGuard> guard_;
  std::unique_ptr<Nav2Executor> nav_;
  std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_;
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> transport_;
  RobotState robot_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconcile_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::deque<std::function<void()>> callbacks_;
  std::string map_, serial_, manufacturer_, error_;
  bool commissioned_=false, connected_=false, online_=false;
  Json localize_action_=nullptr;
  RobotState::Clock::time_point localization_deadline_{};
  Pose localization_target_{};
  std::string localization_id_;
  double max_age_=2, tolerance_=0.12, max_speed_=0.05, max_turn_=0.25;
  double physical_height_=-1;
  double edge_speed_=0.05, edge_turn_=0.25;
  void enqueue(std::function<void()> callback);
  void tick();
  void receive_order(const Json& order);
  void receive_actions(const Json& actions);
  void publish_state();
  void connection(bool online);
  void action_state(const std::string& id,const std::string& type,const std::string& status,const std::string& reason="");
};
}
