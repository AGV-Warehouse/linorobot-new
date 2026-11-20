#pragma once
#include <string>
#include <nlohmann/json.hpp>
namespace fleet {
using Json=nlohmann::json;
// Typed VDA 2.0 actions this profile runs at an order node (openTCS sends them
// for a destination operation other than NOP). This is the plug-in point for
// real load handling: today pick/drop mean "operator loads/unloads the robot
// and confirms"; a lift or docking module replaces that implementation later.
enum class ActionKind {
  WaitForTrigger,  // wait at the station until an operator confirms
  Pick,            // operator puts the load on, then confirms
  Drop,            // operator takes the load off, then confirms
  StartCharging,   // ask the charger to start; finished once the battery reports charging
  StopCharging,    // ask the charger to stop; finished once charging has stopped
  Unsupported
};
ActionKind action_kind(const std::string& type);
// Actions that end only when an operator confirms (button, portal or service).
bool needs_operator(ActionKind kind);
// Throws std::invalid_argument for unsupported types, bad ids/blocking types or duplicates.
void validate_node_actions(const Json& actions);
// VDA terminal states.
inline bool terminal_status(const std::string& status) {return status=="FINISHED" || status=="FAILED";}
}
