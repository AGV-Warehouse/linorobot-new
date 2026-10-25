#pragma once
#include <nlohmann/json.hpp>
#include "linorobot2_fleet_bridge/execution_journal.hpp"
namespace fleet {
using Json=nlohmann::json;
// Deliberately narrow VDA 2.0 profile: straight edges, no transport/load actions.
// Validation and durable execution ownership live here, not in a demo Adapter.
class OrderBook {
public:
  explicit OrderBook(Journal& journal);
  void accept(const Json& order,const std::string& map_id);
  Json next() const;
  void complete();
  void cancel();
  void persist();
  Json snapshot() const;
  Json actions=Json::array();
  Json order=Json::object();
  Json last_message=Json::object();
  std::size_t index=0;
  std::string last_node;
  unsigned last_sequence=0;
  bool paused=true, blocked=false;
private:
  Journal& journal_;
};
}
