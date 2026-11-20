#include "linorobot2_fleet_bridge/node_actions.hpp"
#include <set>
#include <stdexcept>
namespace fleet {
ActionKind action_kind(const std::string& type) {
  if(type=="waitForTrigger") return ActionKind::WaitForTrigger;
  if(type=="pick") return ActionKind::Pick;
  if(type=="drop") return ActionKind::Drop;
  if(type=="startCharging") return ActionKind::StartCharging;
  if(type=="stopCharging") return ActionKind::StopCharging;
  return ActionKind::Unsupported;
}
bool needs_operator(ActionKind kind) {
  return kind==ActionKind::WaitForTrigger || kind==ActionKind::Pick || kind==ActionKind::Drop;
}
void validate_node_actions(const Json& actions) {
  auto fail=[](const std::string& why){throw std::invalid_argument(why);};
  if(!actions.is_array() || actions.size()>4) fail("Too many node actions");
  std::set<std::string> ids;
  for(const auto& action:actions) {
    if(!action.is_object() || !action.contains("actionId") || !action["actionId"].is_string() ||
       !action.contains("actionType") || !action["actionType"].is_string()) fail("Invalid node action");
    const auto id=action["actionId"].get<std::string>();
    if(id.empty() || id.size()>128 || !ids.insert(id).second) fail("Invalid or duplicate node action ID");
    if(action_kind(action["actionType"].get<std::string>())==ActionKind::Unsupported)
      fail("Node action unsupported by this profile: "+action["actionType"].get<std::string>());
    const auto blocking=action.value("blockingType",std::string("HARD"));
    if(blocking!="NONE" && blocking!="SOFT" && blocking!="HARD") fail("Invalid blocking type");
  }
}
}
