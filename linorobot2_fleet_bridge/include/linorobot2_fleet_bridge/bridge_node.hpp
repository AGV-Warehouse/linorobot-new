#pragma once
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vda5050_core/execution/protocol_adapter.hpp>
#include "linorobot2_fleet_bridge/nav2_executor.hpp"
#include "linorobot2_fleet_bridge/robot_state.hpp"
#include "linorobot2_fleet_bridge/vda_actions.hpp"
#include "linorobot2_fleet_bridge/node_actions.hpp"
#include "linorobot2_fleet_bridge/mqtt_transport.hpp"
#include "linorobot2_fleet_bridge/runtime_policy.hpp"
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
  std::shared_ptr<MqttTransport> transport_;
  RobotState robot_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconcile_;
  // Station actions: a physical button (or the portal via VDA instant action) confirms.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr operator_confirm_;
  // Charger request for the robot's charging contacts (the sim models the charger).
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charging_request_;
  bool charging_requested_=false;
  std::map<std::string,RobotState::Clock::time_point> charging_deadlines_;
  double charging_timeout_=30;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_update_;
  RobotState::Clock::time_point next_nomotion_update_{}, nomotion_response_deadline_{};
  bool nomotion_pending_=false;
  std::mutex mutex_;
  std::deque<std::function<void()>> callbacks_;
  bool callback_overflow_=false;
  std::string map_, serial_, manufacturer_, error_;
  bool commissioned_=false, connected_=false, online_=false;
  Json localize_action_=nullptr;
  RobotState::Clock::time_point localization_deadline_{};
  Pose localization_target_{};
  LocalizationConfirmation localization_confirmation_;
  std::int64_t transform_stamp_=0;
  RobotState::Clock::time_point navigation_deadline_{};
  double navigation_timeout_=120;
  unsigned state_ticks_=0;
  std::string logged_fault_;
  std::map<std::string,RobotState::Clock::time_point> action_deadlines_;
  std::string localization_id_;
  double max_age_=2, tolerance_=0.12, max_speed_=0.05, max_turn_=0.25;
  double max_acceleration_=0.05, max_deceleration_=0.1;
  double initial_pose_covariance_xy_=0.25, initial_pose_covariance_yaw_=0.0685;
  double physical_height_=-1;
  double edge_speed_=0.05, edge_turn_=0.25;
  void enqueue(std::function<void()> callback);
  void tick();
  void receive_order(const Json& order);
  void receive_actions(const Json& actions);
  void publish_state();
  void connection(bool online);
  void stop_fault(const std::string& code);
  void log_event(const std::string& category,const std::string& code,const Json& fields=Json::object());
  void action_state(const std::string& id,const std::string& type,const std::string& status,const std::string& reason="",bool deadline=true);
  // Starts/advances typed actions of the last reached node; true while they still block motion.
  bool run_node_actions();
  // Finishes the station action waiting for an operator; returns an error text or "".
  std::string confirm_operator(const std::string& action_id);
  void fail_node_actions(const std::string& reason);
  void request_charging(bool on);
};
}
