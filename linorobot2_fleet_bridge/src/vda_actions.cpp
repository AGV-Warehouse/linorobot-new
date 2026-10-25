#include "linorobot2_fleet_bridge/vda_actions.hpp"
#include <stdexcept>
#include <cmath>
namespace fleet {
OrderBook::OrderBook(Journal& journal):journal_(journal) {
  if(auto saved=journal_.get("execution")) {
    const auto j=Json::parse(*saved);
    order=j.at("order"); index=j.at("index"); actions=j.at("actions");
    last_message=j.value("lastMessage",order);
    last_node=j.at("lastNodeId"); last_sequence=j.at("lastNodeSequenceId");
    blocked=!order.empty();
  }
  // Even previously-idle restarts need local reconciliation before motion.
  paused=true;
}
void OrderBook::accept(const Json& incoming,const std::string& map_id) {
  auto fail=[](const std::string& why){throw std::invalid_argument(why);};
  if(blocked) fail("Restart/failure requires local reconciliation");
  if(!incoming.contains("orderId") || !incoming["orderId"].is_string() || incoming["orderId"].get<std::string>().empty() ||
      !incoming.contains("orderUpdateId") || !incoming["orderUpdateId"].is_number_integer() || incoming["orderUpdateId"].get<long long>()<0) fail("Invalid order identity");
  const std::string id=incoming.at("orderId");
  if(journal_.get("cancelled:"+id)) fail("Cancelled order cannot be replayed");
  if(journal_.get("completed:"+id) && (order.empty() || order.at("orderId")!=id))
    fail("Previously completed order cannot be replayed after a different order");
  const auto& nodes=incoming.at("nodes"); const auto& edges=incoming.at("edges");
  if(!nodes.is_array() || nodes.empty() || nodes.size()>100 || !edges.is_array() || edges.size()+1!=nodes.size()) fail("Invalid graph size");
  if(nodes.front().at("sequenceId").get<unsigned>()%2) fail("Node sequence must be even");
  bool horizon=false;
  for(std::size_t i=0;i<nodes.size();++i) {
    const auto& node=nodes[i]; const auto& pos=node.at("nodePosition");
    if(node.at("nodeId").get<std::string>().empty()) fail("Empty node ID");
    if(node.at("sequenceId").get<unsigned>()!=nodes.front().at("sequenceId").get<unsigned>()+2*i) fail("Nonconsecutive sequence");
    if(!node.value("actions",Json::array()).empty()) fail("Node actions unsupported by this profile");
    if(pos.at("mapId")!=map_id) fail("Wrong map");
    for(const auto* axis:{"x","y"}) if(!std::isfinite(pos.at(axis).get<double>())) fail("Nonfinite position");
    if(pos.contains("theta") && !std::isfinite(pos.at("theta").get<double>())) fail("Nonfinite heading");
    const bool released=node.at("released");
    if(horizon && released) fail("Released base after horizon");
    horizon=horizon || !released;
    if(i) {
      const auto& e=edges[i-1];
      if(e.at("sequenceId").get<unsigned>()!=node.at("sequenceId").get<unsigned>()-1 ||
         e.at("startNodeId")!=nodes[i-1].at("nodeId") || e.at("endNodeId")!=node.at("nodeId") ||
         e.at("released").get<bool>()!=released) fail("Invalid edge connectivity/release");
      if(e.contains("trajectory") || !e.value("actions",Json::array()).empty() ||
         e.contains("orientation") || e.contains("direction")) fail("Only straight unconstrained edges supported");
    }
  }
  if(!nodes.front().at("released").get<bool>()) fail("First node is not released");
  if(!order.empty() && order.at("orderId")==id) {
    const auto update=incoming.at("orderUpdateId").get<unsigned>();
    const auto current=order.at("orderUpdateId").get<unsigned>();
    if(update<current) fail("Outdated order update");
    if(update==current) {
      Json a=last_message,b=incoming; a.erase("headerId"); a.erase("timestamp"); b.erase("headerId"); b.erase("timestamp");
      if(a!=b) fail("Same update ID with different content");
      return;
    }
    const auto& previous=order.at("nodes");
    std::size_t last_released=0;
    for(std::size_t i=0;i<previous.size() && previous[i].at("released").get<bool>();++i) last_released=i;
    if(nodes.front().at("sequenceId")!=previous[last_released].at("sequenceId") ||
       nodes.front().at("nodeId")!=previous[last_released].at("nodeId") ||
       nodes.front().at("nodePosition")!=previous[last_released].at("nodePosition"))
      fail("Order update must stitch at the previous decision point");
    Json merged=incoming;
    merged["nodes"]=Json::array(); merged["edges"]=Json::array();
    for(std::size_t i=0;i<last_released;++i) merged["nodes"].push_back(previous[i]);
    for(const auto& n:nodes) merged["nodes"].push_back(n);
    const auto decision=previous[last_released].at("sequenceId").get<unsigned>();
    for(const auto& e:order.at("edges")) if(e.at("sequenceId").get<unsigned>()<decision) merged["edges"].push_back(e);
    for(const auto& e:edges) merged["edges"].push_back(e);
    order=merged;
  } else {
    if(!order.empty() && index<order.at("nodes").size()) fail("Active order must be cancelled first");
    if(nodes.front().at("sequenceId")!=0) fail("New order must start at sequence zero");
    order=incoming; index=0; actions=Json::array();
  }
  last_message=incoming; persist();
}
Json OrderBook::next() const {
  if(order.empty() || index>=order.at("nodes").size() || !order.at("nodes")[index].at("released").get<bool>()) return nullptr;
  return order.at("nodes")[index];
}
void OrderBook::complete() {
  const auto node=next();
  if(node.is_null()) throw std::logic_error("No released node");
  last_node=node.at("nodeId"); last_sequence=node.at("sequenceId"); ++index;
  if(index==order.at("nodes").size()) journal_.put("completed:"+order.at("orderId").get<std::string>(),"true");
  persist();
}
void OrderBook::cancel() {
  if(!order.empty()) journal_.put("cancelled:"+order.at("orderId").get<std::string>(),"true");
  order=Json::object(); index=0; blocked=false; persist();
}
void OrderBook::persist() { journal_.put("execution",snapshot().dump()); }
Json OrderBook::snapshot() const {
  return {{"order",order},{"lastMessage",last_message},{"index",index},{"actions",actions},{"lastNodeId",last_node},
    {"lastNodeSequenceId",last_sequence},{"paused",paused}};
}
}
