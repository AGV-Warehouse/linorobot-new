#pragma once
#include <cmath>
#include <stdexcept>
#include <vda5050_core/json_utils/serialization.hpp>
#include <vda5050_core/types/instant_actions.hpp>
namespace fleet {
// The pinned core models parameter values as strings. VDA 2.0 initPosition
// coordinates may be JSON numbers. Adapt only its validation copy; execution
// and journal identity checks always retain the original wire payload.
inline void validate_instant_actions(const nlohmann::json& raw) {
  auto typed=raw;
  for(auto& action:typed.at("actions")) {
    if(action.at("actionType")!="initPosition") continue;
    for(auto& parameter:action.at("actionParameters")) {
      const auto key=parameter.at("key").get<std::string>();
      auto& value=parameter.at("value");
      if((key=="x" || key=="y" || key=="theta") && value.is_number()) {
        if(!std::isfinite(value.get<double>())) throw std::invalid_argument("Nonfinite initPosition coordinate");
        value=value.dump();
      }
    }
  }
  (void)typed.get<vda5050_core::types::InstantActions>();
}
}
