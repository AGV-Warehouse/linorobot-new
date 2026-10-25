#include "linorobot2_fleet_bridge/bridge_node.hpp"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <regex>
#include <mqtt/async_client.h>
#include <vda5050_core/types/instant_actions.hpp>
namespace fleet {
namespace vt=vda5050_core::types;
namespace {
std::string environment(const char* name) {const auto* v=std::getenv(name);return v?v:"";}
std::string fault_category(const std::string& code) {
  if(code.rfind("MQTT",0)==0) return "transport";
  if(code.rfind("NAV",0)==0 || code=="ACTION_TIMEOUT") return "execution";
  if(code=="ORDER_REJECTED" || code=="ACTION_REJECTED" || code=="INGRESS_OVERFLOW" ||
     code=="PAYLOAD_TOO_LARGE" || code=="RETAINED_COMMAND_REJECTED" || code=="INVALID_MESSAGE") return "protocol";
  return "safety";
}
Pose node_pose(const Json& node) {const auto& p=node.at("nodePosition");return {p.at("x"),p.at("y"),p.value("theta",0.0)};}
}
BridgeNode::BridgeNode():Node("linorobot2_fleet_bridge"),tf_(get_clock()),listener_(tf_) {
  commissioned_=declare_parameter("commissioned",false);
  map_=declare_parameter("map_id",std::string("map"));
  serial_=declare_parameter("serial_number",std::string("AGV-01"));
  manufacturer_=declare_parameter("manufacturer",std::string("AGVWarehouse"));
  const std::regex topic_identity("[A-Za-z0-9_-]{1,64}");
  if(!std::regex_match(serial_,topic_identity) || !std::regex_match(manufacturer_,topic_identity))
    throw std::invalid_argument("Invalid MQTT manufacturer/serial identity");
  physical_height_=declare_parameter("physical_height_m",-1.0);
  max_age_=declare_parameter("sensor_max_age_s",2.0);
  tolerance_=declare_parameter("arrival_tolerance_m",0.12);
  max_speed_=declare_parameter("max_speed",0.05); max_turn_=declare_parameter("max_turn",0.25);
  navigation_timeout_=declare_parameter("navigation_timeout_s",120.0);
  const auto width=declare_parameter("centre_corridor_half_width",0.15);
  if(!std::isfinite(max_age_+tolerance_+width+max_speed_+max_turn_+navigation_timeout_) ||
     max_age_<=0 || tolerance_<=0 || width<=tolerance_ || max_speed_<=0 || max_turn_<=0 || navigation_timeout_<=0)
    throw std::invalid_argument("Invalid motion/sensor limits");
  auto path=declare_parameter("journal_path",std::string(""));
  if(path.empty()) path=environment("HOME")+"/.ros/linorobot2_fleet/"+serial_+"/execution.sqlite";
  journal_=std::make_unique<Journal>(path); book_=std::make_unique<OrderBook>(*journal_);
  guard_=std::make_unique<RouteGuard>(width); nav_=std::make_unique<Nav2Executor>(*this,*guard_);
  output_=create_publisher<geometry_msgs::msg::Twist>("fleet/cmd_vel_authorized",10);
  initial_pose_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",10);
  auto qos=rclcpp::SensorDataQoS();
  localization_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",qos,
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg){
      if(localize_action_.is_null() || msg->header.frame_id!="map") return;
      const auto stamp=rclcpp::Time(msg->header.stamp);
      const double age=(now()-stamp).seconds();
      if(age<0 || age>max_age_) return;
      const auto& p=msg->pose.pose;
      localization_confirmation_.observe(stamp.nanoseconds(),{p.position.x,p.position.y,
        2*std::atan2(p.orientation.z,p.orientation.w)},msg->pose.covariance[0],msg->pose.covariance[7],
        msg->pose.covariance[35],tolerance_);
    });
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odom",qos,[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
    const auto age=(now()-rclcpp::Time(msg->header.stamp)).seconds();
    if(age < -0.5 || age>max_age_) return;
    robot_.vx=msg->twist.twist.linear.x;robot_.wz=msg->twist.twist.angular.z;robot_.velocity_at=RobotState::Clock::now();});
  battery_sub_=create_subscription<sensor_msgs::msg::BatteryState>("battery_state",qos,[this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg){
    const auto age=(now()-rclcpp::Time(msg->header.stamp)).seconds();
    if(age < -0.5 || age>max_age_) return;
    if(msg->present && std::isfinite(msg->percentage) && msg->percentage>=0 && msg->percentage<=1) {
      robot_.battery=100*msg->percentage; robot_.battery_at=RobotState::Clock::now();
      robot_.charging=msg->power_supply_status==sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else robot_.battery.reset();
  });
  safety_sub_=create_subscription<std_msgs::msg::String>("fleet/safety_state",qos,[this](std_msgs::msg::String::ConstSharedPtr msg){
    try {
      const auto state=Json::parse(msg->data);
      const auto stop=state.at("eStop").get<std::string>();
      if(stop!="NONE" && stop!="AUTOACK" && stop!="MANUAL" && stop!="REMOTE") throw std::invalid_argument("Invalid eStop");
      robot_.field_violation=state.at("fieldViolation").get<bool>();robot_.e_stop=stop;
      robot_.safety_ok=stop=="NONE" && !robot_.field_violation;
      robot_.safety_at=RobotState::Clock::now();
    } catch(const std::exception&) {robot_.safety_ok=false;robot_.e_stop.clear();}
  });
  velocity_sub_=create_subscription<geometry_msgs::msg::Twist>("fleet/cmd_vel_requested",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr msg){
    geometry_msgs::msg::Twist out;
    const bool permit=commissioned_ && connected_ && transport_->connected() && localize_action_.is_null() &&
      !book_->paused && !book_->blocked && nav_->executing() && robot_.ready(max_age_);
    Pose projected=robot_.pose;
    const double braking=std::abs(msg->linear.x)*0.5 + msg->linear.x*msg->linear.x/0.2;
    projected.x+=std::copysign(braking,msg->linear.x)*std::cos(projected.yaw);
    projected.y+=std::copysign(braking,msg->linear.x)*std::sin(projected.yaw);
    if(permit && std::isfinite(msg->linear.x) && std::isfinite(msg->angular.z) && guard_->contains(projected)) {
      out.linear.x=std::clamp(msg->linear.x,-edge_speed_,edge_speed_);
      out.angular.z=std::clamp(msg->angular.z,-edge_turn_,edge_turn_);
    }
    output_->publish(out);
  });
  reconcile_=create_service<std_srvs::srv::Trigger>("fleet/reconcile",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    response->success=commissioned_ && connected_ && transport_->connected() && localize_action_.is_null() &&
      robot_.ready(max_age_) && robot_.stopped() && !nav_->busy();
    if(response->success) {
      const auto pending=book_->actions;
      for(const auto& a:pending) if(a.at("actionStatus")=="RUNNING") action_state(a.at("actionId"),a.at("actionType"),"FAILED","Locally reconciled");
      book_->cancel();book_->paused=false;book_->persist();error_.clear();log_event("audit","LOCAL_RECONCILED");}
    response->message=response->success?"Old order discarded locally; ready for a fresh order":"Commissioning, fresh sensors and stationary robot required";
  });
  const auto broker=declare_parameter("mqtt_broker",std::string("ssl://localhost:8883"));
  transport_=std::make_shared<MqttTransport>(broker,"fleet-"+serial_);
  protocol_=vda5050_core::execution::ProtocolAdapter::make(transport_,"uagv","2.0.0",manufacturer_,serial_);
  vt::Connection will{};will.connection_state=vt::ConnectionState::CONNECTIONBROKEN;protocol_->set_will(will,1,true);
  transport_->set_connected_callback([this](auto){enqueue([this]{connected_=true;online_=false;log_event("transport","MQTT_CONNECTED");});});
  transport_->set_connection_lost_callback([this](auto){enqueue([this]{connected_=false;online_=false;stop_fault("MQTT_DISCONNECTED");});});
  transport_->set_fault_callback([this](auto code){enqueue([this,code]{stop_fault(code);});});
  // Validate with the pinned VDA types, but retain raw optional fields for profile rejection.
  const auto prefix="uagv/v2/"+manufacturer_+"/"+serial_+"/";
  transport_->subscribe(prefix+"order",[this](auto,auto payload){enqueue([this,payload]{
    try{const auto raw=Json::parse(payload);(void)raw.get<vt::Order>();receive_order(raw);}
    catch(const std::exception&){stop_fault("ORDER_REJECTED");}});},0);
  transport_->subscribe(prefix+"instantActions",[this](auto,auto payload){enqueue([this,payload]{
    try{const auto raw=Json::parse(payload);(void)raw.get<vt::InstantActions>();receive_actions(raw);}
    catch(const std::exception&){stop_fault("ACTION_REJECTED");}});},0);
  protocol_->connect(); connected_=protocol_->connected();
  timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  log_event("system","BRIDGE_STARTED",{{"profile","linorobot-straight/1.0"},{"commissioned",commissioned_}});
}
BridgeNode::~BridgeNode() {transport_->stop_callbacks();nav_->cancel();output_->publish(geometry_msgs::msg::Twist());
  transport_->set_connected_callback({});transport_->set_connection_lost_callback({});
  try{connection(false);protocol_->unsubscribe_all();protocol_->disconnect();}catch(...){} }
void BridgeNode::enqueue(std::function<void()> f) {std::lock_guard<std::mutex> lock(mutex_);
  if(callbacks_.size()<32) callbacks_.push_back(std::move(f));else callback_overflow_=true;}
void BridgeNode::log_event(const std::string& category,const std::string& code,const Json& fields) {
  Json value={{"component","fleet-bridge"},{"category",category},{"code",code},{"robotId",serial_},
    {"timestampUnixNs",now().nanoseconds()},{"orderId",book_->order.value("orderId",std::string(""))},{"fields",fields}};
  RCLCPP_INFO(get_logger(),"%s",value.dump().c_str());
}
void BridgeNode::stop_fault(const std::string& code) {
  output_->publish(geometry_msgs::msg::Twist());if(nav_->busy()) nav_->cancel();
  const bool changed=!book_->blocked || error_.empty();
  book_->blocked=true;book_->paused=true;
  // Preserve the first fault until reconciliation; avoid cascaded fault log storms.
  if(changed) {error_=code;logged_fault_=code;book_->persist();log_event(fault_category(code),code);}
}
void BridgeNode::connection(bool on) {
  vt::Connection c{};c.connection_state=on?vt::ConnectionState::ONLINE:vt::ConnectionState::OFFLINE;
  protocol_->publish(c,1,true);online_=on;
}
void BridgeNode::receive_order(const Json& order) {
  if(order.at("manufacturer")!=manufacturer_ || order.at("serialNumber")!=serial_ || order.at("version")!="2.0.0")
    throw std::invalid_argument("Order identity/version mismatch");
  if(!commissioned_ || !connected_ || !localize_action_.is_null() || !robot_.ready(max_age_)) throw std::invalid_argument("Robot not commissioned/ready");
  if(std::any_of(book_->actions.begin(),book_->actions.end(),[](const auto& a){return a.at("actionType")=="cancelOrder" && a.at("actionStatus")=="RUNNING";}))
    throw std::invalid_argument("Cancellation must complete before a new order/update");
  if(order.at("nodes").empty()) throw std::invalid_argument("Empty order");
  for(const auto& node:order.at("nodes")) {
    const auto& position=node.at("nodePosition");
    for(const auto* field:{"allowedDeviationXY","allowedDeviationTheta"}) if(position.contains(field)) {
      const double value=position.at(field).get<double>();
      if(!std::isfinite(value) || value<(std::string(field)=="allowedDeviationXY"?tolerance_:0.15))
        throw std::invalid_argument("Node tolerance tighter than commissioned profile");
    }
  }
  for(const auto& edge:order.at("edges")) {
    for(const auto* key:{"maxSpeed","maxRotationSpeed"}) if(edge.contains(key) &&
      (!edge.at(key).is_number() || !std::isfinite(edge.at(key).get<double>()) || edge.at(key).get<double>()<=0))
      throw std::invalid_argument("Invalid edge speed limit");
    if(edge.contains("minHeight") || edge.contains("maxHeight"))
      throw std::invalid_argument("Height-constrained edges unsupported by this profile");
  }
  if(book_->order.empty()) {
    const auto start=node_pose(order.at("nodes").front());
    if(std::hypot(start.x-robot_.pose.x,start.y-robot_.pose.y)>tolerance_)
      throw std::invalid_argument("First node is not current position");
  }
  book_->accept(order,map_);error_.clear();log_event("execution","ORDER_ACCEPTED",{{"orderUpdateId",order.at("orderUpdateId")}});
}
void BridgeNode::action_state(const std::string& id,const std::string& type,const std::string& status,const std::string& reason) {
  if(status=="RUNNING") action_deadlines_[id]=RobotState::Clock::now()+std::chrono::seconds(10);
  else action_deadlines_.erase(id);
  Json value={{"actionId",id},{"actionType",type},{"actionStatus",status},{"resultDescription",reason}};
  bool found=false;
  for(auto& action:book_->actions) if(action.at("actionId")==id) {action=value;found=true;break;}
  if(!found) book_->actions.push_back(value);
  if(book_->actions.size()>100) book_->actions.erase(book_->actions.begin());
  journal_->put("action:"+id,value.dump());book_->persist();
  log_event("execution","ACTION_STATE",{{"actionId",id},{"actionType",type},{"status",status}});
}
void BridgeNode::receive_actions(const Json& message) {
  if(message.at("manufacturer")!=manufacturer_ || message.at("serialNumber")!=serial_ || message.at("version")!="2.0.0")
    throw std::invalid_argument("Action identity/version mismatch");
  if(!message.at("actions").is_array() || message.at("actions").size()>100) throw std::invalid_argument("Action count exceeded");
  for(const auto& action:message.at("actions")) {
    const auto id=action.at("actionId").get<std::string>(), type=action.at("actionType").get<std::string>();
    if(id.empty() || id.size()>128 || type.size()>64) throw std::invalid_argument("Invalid action identity");
    const auto exists=std::find_if(book_->actions.begin(),book_->actions.end(),[&](const auto& a){return a.at("actionId")==id;});
    if(exists==book_->actions.end() && book_->actions.size()>=100) {
      const auto terminal=std::find_if(book_->actions.begin(),book_->actions.end(),[](const auto& a){return a.at("actionStatus")=="FINISHED" || a.at("actionStatus")=="FAILED";});
      if(terminal==book_->actions.end()) throw std::invalid_argument("Pending action capacity exceeded");
      book_->actions.erase(terminal);
    }
    if(auto saved=journal_->get("action:"+id)) {
      if(auto original=journal_->get("actionRequest:"+id); original && *original!=action.dump())
        throw std::invalid_argument("Action ID reused with different content");
      auto previous=Json::parse(*saved);
      if(previous.at("actionType")!=type) throw std::invalid_argument("Action ID reused with different type");
      if(previous.at("actionStatus")=="RUNNING" && !action_deadlines_.count(id)) {
        action_state(id,type,"FAILED","Process interrupted; inspect and reconcile before new intent");continue;
      }
      // Persistent terminal result or pending record; never execute it twice.
      bool found=false;for(auto& a:book_->actions) if(a.at("actionId")==id){a=previous;found=true;}
      if(!found) book_->actions.push_back(previous);continue;
    }
    journal_->put("actionRequest:"+id,action.dump());
    action_state(id,type,"RUNNING");
    try {
    if(type=="stateRequest") {action_state(id,type,"FINISHED");}
    else if(type=="factsheetRequest") {
      if(!std::isfinite(physical_height_) || physical_height_<=0) {
        action_state(id,type,"FAILED","Set measured physical_height_m before factsheet commissioning");continue;
      }
      vt::Factsheet facts{};
      facts.type_specification.series_name="linorobot2 straight-route profile";
      facts.type_specification.agv_kinematic=vt::AGVKinematic::DIFF;
      facts.type_specification.agv_class=vt::AGVClass::CARRIER;
      facts.type_specification.max_load_mass=0;
      facts.type_specification.localization_types={"NATURAL"};
      facts.type_specification.navigation_types={"AUTONOMOUS"};
      facts.physical_parameters.speed_max=max_speed_;
      facts.physical_parameters.angular_speed_max=max_turn_;
      facts.physical_parameters.acceleration_max=0.05;facts.physical_parameters.deceleration_max=0.1;
      facts.physical_parameters.width=0.55;facts.physical_parameters.length=0.55;
      facts.physical_parameters.height_min=physical_height_;facts.physical_parameters.height_max=physical_height_;
      facts.protocol_limits.max_array_lens.order_nodes=100;facts.protocol_limits.max_array_lens.order_edges=99;
      protocol_->publish(facts,0);action_state(id,type,"FINISHED");
    }
    else if(type=="startPause" || type=="cancelOrder") {
      book_->paused=true;nav_->cancel();book_->persist();
      // tick waits for real odometry to confirm stationary before FINISHED.
    } else if(type=="stopPause") {
      const bool pending_stop=std::any_of(book_->actions.begin(),book_->actions.end(),[](const auto& a){
        return a.at("actionStatus")=="RUNNING" && (a.at("actionType")=="startPause" || a.at("actionType")=="cancelOrder");});
      if(!commissioned_ || !connected_ || pending_stop || nav_->busy() || !robot_.stopped() ||
         !localize_action_.is_null() || !robot_.ready(max_age_) || book_->blocked) action_state(id,type,"FAILED","Reconciliation/fresh sensors and completed stop required");
      else {book_->paused=false;book_->persist();action_state(id,type,"FINISHED");}
    } else if(type=="initPosition") {
      if(nav_->busy() || !robot_.stopped() || !localize_action_.is_null() || !book_->next().is_null() ||
         std::chrono::duration<double>(RobotState::Clock::now()-robot_.velocity_at).count()>max_age_)
        {action_state(id,type,"FAILED","Fresh stationary odometry and no pending route required");continue;}
      Json params=Json::object();for(const auto& p:action.at("actionParameters")) {
        const auto key=p.at("key").get<std::string>();if(params.contains(key)) throw std::invalid_argument("Duplicate action parameter");
        params[key]=p.at("value");}
      if(params.at("mapId")!=map_){action_state(id,type,"FAILED","Wrong map");continue;}
      auto number=[](const Json& j){return j.is_string()?std::stod(j.get<std::string>()):j.get<double>();};
      localization_target_={number(params.at("x")),number(params.at("y")),number(params.at("theta"))};
      localization_id_=params.at("lastNodeId").get<std::string>();
      if(localization_id_.empty() || localization_id_.size()>128) throw std::invalid_argument("Invalid lastNodeId");
      if(!std::isfinite(localization_target_.x) || !std::isfinite(localization_target_.y) || !std::isfinite(localization_target_.yaw)) {action_state(id,type,"FAILED","Invalid pose");continue;}
      geometry_msgs::msg::PoseWithCovarianceStamped pose;pose.header.frame_id="map";pose.header.stamp=now();
      pose.pose.pose.position.x=localization_target_.x;pose.pose.pose.position.y=localization_target_.y;
      pose.pose.pose.orientation.z=std::sin(localization_target_.yaw/2);pose.pose.pose.orientation.w=std::cos(localization_target_.yaw/2);
      pose.pose.covariance[0]=0.25;pose.pose.covariance[7]=0.25;pose.pose.covariance[35]=0.0685;
      localize_action_=action; localization_deadline_=RobotState::Clock::now()+std::chrono::seconds(10);
      localization_confirmation_.begin(rclcpp::Time(pose.header.stamp).nanoseconds(),localization_target_);initial_pose_->publish(pose);
    } else {action_state(id,type,"FAILED","Action unsupported by commissioned straight-route profile");}
    } catch(const std::exception& e) {action_state(id,type,"FAILED",e.what());}
  }
}
void BridgeNode::tick() {
  std::deque<std::function<void()>> work;
  bool overflow=false;
  {std::lock_guard<std::mutex> lock(mutex_);work.swap(callbacks_);overflow=callback_overflow_;callback_overflow_=false;}
  if(overflow) {work.clear();stop_fault("INGRESS_OVERFLOW");}
  for(auto& f:work) {try{f();}catch(const std::exception&){stop_fault("CALLBACK_FAILED");}}
  if(!connected_ || !transport_->connected()) {connected_=false;stop_fault("MQTT_DISCONNECTED");}
  if(nav_->busy() && RobotState::Clock::now()>navigation_deadline_) stop_fault("NAVIGATION_TIMEOUT");
  try {
    const auto transform=tf_.lookupTransform("map","base_footprint",tf2::TimePointZero);
    const auto tf_age=(now()-rclcpp::Time(transform.header.stamp)).seconds();
    if(tf_age>=-0.5 && tf_age<=max_age_) {
      transform_stamp_=rclcpp::Time(transform.header.stamp).nanoseconds();
      robot_.pose={transform.transform.translation.x,transform.transform.translation.y,
        2*std::atan2(transform.transform.rotation.z,transform.transform.rotation.w)};
      robot_.pose_at=RobotState::Clock::now();robot_.localized=true;
    } else robot_.localized=false;
  }catch(const tf2::TransformException&) {robot_.localized=false;}
  const bool ready=commissioned_ && connected_ && robot_.ready(max_age_) && localize_action_.is_null();
  if(!ready || book_->paused || book_->blocked) {
    if(nav_->busy()) {if(!ready) stop_fault("SENSOR_OR_SAFETY_LOST");else nav_->cancel();}
    output_->publish(geometry_msgs::msg::Twist());
  }
  if(!localize_action_.is_null()) {
    const auto a=localize_action_;const auto id=a.at("actionId").get<std::string>();
    const double yaw_error=std::atan2(std::sin(robot_.pose.yaw-localization_target_.yaw),std::cos(robot_.pose.yaw-localization_target_.yaw));
    if(robot_.localized && localization_confirmation_.confirmed(transform_stamp_,now().nanoseconds(),max_age_) &&
       std::hypot(robot_.pose.x-localization_target_.x,robot_.pose.y-localization_target_.y)<=tolerance_ && std::abs(yaw_error)<0.15) {
      book_->last_node=localization_id_;book_->persist();action_state(id,"initPosition","FINISHED");localize_action_=nullptr;
    } else if(RobotState::Clock::now()>localization_deadline_) {action_state(id,"initPosition","FAILED","Localization confirmation timed out");localize_action_=nullptr;}
  }
  if(robot_.ready(max_age_) && robot_.stopped() && !nav_->busy()) {
    const auto actions=book_->actions;
    for(const auto& a:actions) if(a.at("actionStatus")=="RUNNING") {
      const auto type=a.at("actionType").get<std::string>();
      const auto pending=action_deadlines_.find(a.at("actionId").get<std::string>());
      if(pending==action_deadlines_.end() || robot_.velocity_at<=pending->second-std::chrono::seconds(10)) continue;
      if(type=="cancelOrder") {const bool blocked=book_->blocked;book_->cancel();book_->blocked=blocked;book_->persist();action_state(a.at("actionId"),type,"FINISHED");}
      if(type=="startPause") action_state(a.at("actionId"),type,"FINISHED");
    }
  }
  const auto pending_actions=book_->actions;
  for(const auto& a:pending_actions) if(a.at("actionStatus")=="RUNNING") {
    const std::string id=a.at("actionId");const auto deadline=action_deadlines_.find(id);
    if(deadline!=action_deadlines_.end() && RobotState::Clock::now()>deadline->second) {
      action_state(id,a.at("actionType"),"FAILED","Physical confirmation deadline exceeded");stop_fault("ACTION_TIMEOUT");
    }
  }
  if(ready && !book_->paused && !book_->blocked && !nav_->busy()) {
    const auto target=book_->next();
    if(!target.is_null()) {
      const auto pose=node_pose(target);
      if(book_->index==0 && robot_.stopped() && std::hypot(pose.x-robot_.pose.x,pose.y-robot_.pose.y)<=tolerance_) book_->complete();
      else if(book_->index==0) {book_->blocked=true;error_="START_NODE_POSITION_CHANGED";book_->persist();}
      else {
        const auto& edge=book_->order.at("edges")[book_->index-1];
        edge_speed_=std::min(max_speed_,edge.value("maxSpeed",max_speed_));
        edge_turn_=edge.value("rotationAllowed",true)?std::min(max_turn_,edge.value("maxRotationSpeed",max_turn_)):0.0;
        const auto start=node_pose(book_->order.at("nodes")[book_->index-1]);
        guard_->authorize(start,pose);
        if(!guard_->contains(robot_.pose)) {book_->blocked=true;error_="OUTSIDE_AUTHORIZED_ROUTE";book_->persist();}
        else {navigation_deadline_=RobotState::Clock::now()+std::chrono::milliseconds(static_cast<int>(navigation_timeout_*1000));
        const bool heading_required=target.at("nodePosition").contains("theta");
        if(!nav_->start(pose,[this,pose,heading_required](bool ok){
          const double yaw_error=std::atan2(std::sin(pose.yaw-robot_.pose.yaw),std::cos(pose.yaw-robot_.pose.yaw));
          if(ok && robot_.ready(max_age_) && std::hypot(pose.x-robot_.pose.x,pose.y-robot_.pose.y)<=tolerance_ &&
             (!heading_required || std::abs(yaw_error)<0.15)) book_->complete();
          else {book_->blocked=true;error_="NAVIGATION_FAILED";book_->persist();}
        })) {guard_->revoke();stop_fault("NAV2_NOT_READY");}}
      }
    }
  }
  if(error_!=logged_fault_) {logged_fault_=error_;log_event(fault_category(error_),error_.empty()?"FAULT_CLEARED":error_);}
  if(connected_ && ++state_ticks_%20==0) {
    // VDA 2.0 requires numeric battery/safety fields. Do not publish fabricated healthy values.
    if(robot_.battery && !robot_.e_stop.empty()) {if(!online_) connection(true);publish_state();}
  }
}
void BridgeNode::publish_state() {
  Json nodes=Json::array(),edges=Json::array();
  if(!book_->order.empty()) {
    const auto& all=book_->order.at("nodes");
    for(std::size_t i=book_->index;i<all.size();++i) nodes.push_back({{"nodeId",all[i].at("nodeId")},{"sequenceId",all[i].at("sequenceId")},{"released",all[i].at("released")}});
    for(const auto& edge:book_->order.at("edges")) if(edge.at("sequenceId").get<unsigned>()>book_->last_sequence)
      edges.push_back({{"edgeId",edge.at("edgeId")},{"sequenceId",edge.at("sequenceId")},{"released",edge.at("released")}});
  }
  Json errors=Json::array();
  if(!error_.empty() || !robot_.ready(max_age_) || !commissioned_ || book_->blocked)
    errors.push_back({{"errorType",error_.empty()?"ROBOT_NOT_READY":error_},{"errorLevel","FATAL"}});
  Json state={{"headerId",0},{"timestamp","2026-01-01T00:00:00Z"},{"version","2.0.0"},{"manufacturer",manufacturer_},{"serialNumber",serial_},
    {"orderId",book_->order.value("orderId",std::string(""))},{"orderUpdateId",book_->order.value("orderUpdateId",0)},
    {"lastNodeId",book_->last_node},{"lastNodeSequenceId",book_->last_sequence},
    {"nodeStates",nodes},{"edgeStates",edges},{"driving",!robot_.stopped()}, {"paused",book_->paused},
    {"actionStates",book_->actions},{"batteryState",{{"batteryCharge",*robot_.battery},{"charging",robot_.charging}}},
    {"operatingMode","AUTOMATIC"},{"errors",errors},
    {"safetyState",{{"eStop",robot_.e_stop},{"fieldViolation",robot_.field_violation}}},
    {"agvPosition",{{"positionInitialized",robot_.localized && robot_.ready(max_age_)},{"x",robot_.pose.x},{"y",robot_.pose.y},{"theta",robot_.pose.yaw},{"mapId",map_}}},
    {"velocity",{{"vx",robot_.vx},{"vy",0},{"omega",robot_.wz}}}};
  try {protocol_->publish(state.get<vt::State>(),0);}catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"State serialization: %s",e.what());}
}
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<fleet::BridgeNode>();rclcpp::spin(node);node.reset();}
  catch(const std::exception& e){fprintf(stderr,"Fleet bridge: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
