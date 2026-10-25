#include "linorobot2_fleet_bridge/mqtt_transport.hpp"
#include <cstdlib>
namespace fleet {
namespace {std::string env(const char* name) {const auto* v=std::getenv(name);return v?v:"";}}
MqttTransport::MqttTransport(const std::string& broker,const std::string& id):client_(broker,id) {
  client_.set_callback(*this);
  options_.set_clean_session(true);options_.set_keep_alive_interval(5);
  options_.set_connect_timeout(5);options_.set_automatic_reconnect(1,10);
  const auto user=env("FLEET_MQTT_USERNAME"), pass=env("FLEET_MQTT_PASSWORD");
  if(!user.empty()) {options_.set_user_name(user);options_.set_password(pass);}
  if(broker.rfind("ssl://",0)==0) {
    mqtt::ssl_options ssl;const auto ca=env("FLEET_MQTT_CA");
    if(!ca.empty()) ssl.set_trust_store(ca);
    ssl.set_enable_server_cert_auth(true);ssl.set_verify(true);options_.set_ssl(ssl);
  }
}
MqttTransport::~MqttTransport() {stop_callbacks();try{disconnect();}catch(...){} }
void MqttTransport::stop_callbacks() {
  // Wait for any copied callback to finish before its owner is destroyed.
  std::lock_guard<std::mutex> lock(dispatch_mutex_);closing_=true;client_.disable_callbacks();
}
void MqttTransport::set_fault_callback(std::function<void(std::string)> f) {std::lock_guard<std::mutex> l(mutex_);fault_=std::move(f);}
void MqttTransport::connect() {client_.connect(options_)->wait();}
void MqttTransport::disconnect() {if(client_.is_connected()) client_.disconnect()->wait();}
bool MqttTransport::connected() {return client_.is_connected();}
void MqttTransport::publish(const std::string& topic,const std::string& body,int qos,bool retain) {
  try {client_.publish(topic,body.data(),body.size(),qos,retain);}
  catch(...) {std::function<void(std::string)> fault;{std::lock_guard<std::mutex> l(mutex_);fault=fault_;}
    if(fault) fault("MQTT_PUBLISH_FAILED");throw;}
}
void MqttTransport::subscribe(const std::string& topic,MessageHandler handler,int qos) {
  {std::lock_guard<std::mutex> lock(mutex_);subscriptions_[topic]={std::move(handler),qos};}
  if(connected()) client_.subscribe(topic,qos);
}
void MqttTransport::unsubscribe(const std::string& topic) {
  {std::lock_guard<std::mutex> lock(mutex_);subscriptions_.erase(topic);}
  if(connected()) client_.unsubscribe(topic);
}
void MqttTransport::set_will(const std::string& topic,const std::string& body,int qos,bool retain) {options_.set_will(mqtt::will_options(topic,body,qos,retain));}
void MqttTransport::set_connection_lost_callback(ConnectionStateHandler f) {std::lock_guard<std::mutex> l(mutex_);lost_=std::move(f);}
void MqttTransport::set_connected_callback(ConnectionStateHandler f) {std::lock_guard<std::mutex> l(mutex_);connected_=std::move(f);}
void MqttTransport::connected(const std::string& cause) {
  std::lock_guard<std::mutex> dispatch(dispatch_mutex_);if(closing_) return;
  ConnectionStateHandler handler;std::function<void(std::string)> fault;
  try {
    {std::lock_guard<std::mutex> l(mutex_);fault=fault_;for(const auto& entry:subscriptions_) client_.subscribe(entry.first,entry.second.second);handler=connected_;}
    if(handler) handler(cause);
  }catch(...) {if(fault) fault("MQTT_SUBSCRIBE_FAILED");}
}
void MqttTransport::connection_lost(const std::string& cause) {
  std::lock_guard<std::mutex> dispatch(dispatch_mutex_);if(closing_) return;
  ConnectionStateHandler handler;{std::lock_guard<std::mutex> l(mutex_);handler=lost_;}
  if(handler) handler(cause);
}
void MqttTransport::message_arrived(mqtt::const_message_ptr message) {
  std::lock_guard<std::mutex> dispatch(dispatch_mutex_);if(closing_) return;
  MessageHandler handler;std::function<void(std::string)> fault;
  {std::lock_guard<std::mutex> l(mutex_);fault=fault_;auto it=subscriptions_.find(message->get_topic());if(it!=subscriptions_.end()) handler=it->second.first;}
  if(message->is_retained() || message->get_payload().size()>262144) {
    if(fault) fault(message->is_retained()?"RETAINED_COMMAND_REJECTED":"PAYLOAD_TOO_LARGE");return;
  }
  try {if(handler) handler(message->get_topic(),message->to_string());}
  catch(...) {if(fault) fault("INVALID_MESSAGE");}
}
}
