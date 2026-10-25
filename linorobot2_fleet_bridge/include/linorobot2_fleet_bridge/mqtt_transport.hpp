#pragma once
#include <vda5050_core/transport/mqtt_client_interface.hpp>
#include <mqtt/async_client.h>
#include <map>
#include <mutex>
#include <functional>
namespace fleet {
// Owns only MQTT/TLS and bounded raw ingress. No robot behavior here.
class MqttTransport final: public vda5050_core::transport::MqttClientInterface,
                           public virtual mqtt::callback {
  mqtt::async_client client_;
  mqtt::connect_options options_;
  std::mutex mutex_, dispatch_mutex_;
  bool closing_=false;
  std::map<std::string,std::pair<MessageHandler,int>> subscriptions_;
  ConnectionStateHandler lost_, connected_;
  std::function<void(std::string)> fault_;
public:
  MqttTransport(const std::string& broker,const std::string& id);
  ~MqttTransport() override;
  void stop_callbacks();
  void set_fault_callback(std::function<void(std::string)> f);
  void connect() override;
  void disconnect() override;
  bool connected() override;
  void publish(const std::string& topic,const std::string& body,int qos,bool retain=false) override;
  void subscribe(const std::string& topic,MessageHandler handler,int qos) override;
  void unsubscribe(const std::string& topic) override;
  void set_will(const std::string& topic,const std::string& body,int qos,bool retain=true) override;
  void set_connection_lost_callback(ConnectionStateHandler f) override;
  void set_connected_callback(ConnectionStateHandler f) override;
  void connected(const std::string& cause) override;
  void connection_lost(const std::string& cause) override;
  void message_arrived(mqtt::const_message_ptr message) override;
};
}
