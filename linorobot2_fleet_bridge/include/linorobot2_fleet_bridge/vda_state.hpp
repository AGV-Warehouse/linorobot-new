#pragma once
#include <vda5050_core/json_utils/serialization.hpp>
#include <vda5050_core/types/state.hpp>
namespace fleet {
// Build a parseable intermediate header with the pinned core serializer.
// ProtocolAdapter::publish replaces it with the real timestamp and sequence.
inline vda5050_core::types::State state_from_payload(nlohmann::json payload) {
  payload.update(nlohmann::json(vda5050_core::types::Header{}));
  return payload.get<vda5050_core::types::State>();
}
}
