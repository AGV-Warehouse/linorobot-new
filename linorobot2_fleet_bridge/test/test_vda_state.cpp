#include <gtest/gtest.h>
#include "linorobot2_fleet_bridge/vda_state.hpp"
TEST(VdaState, BodyConvertsWithCoreGeneratedIntermediateHeader) {
  namespace vt=vda5050_core::types;
  nlohmann::json payload=vt::State{};
  for(const auto* key:{"headerId","timestamp","version","manufacturer","serialNumber"}) payload.erase(key);
  payload["lastNodeId"]="L-STAGE";
  payload["batteryState"]["batteryCharge"]=75.0;
  const auto state=fleet::state_from_payload(payload);
  EXPECT_EQ(state.last_node_id,"L-STAGE");
  EXPECT_DOUBLE_EQ(state.battery_state.battery_charge,75.0);
  const nlohmann::json serialized=state;
  EXPECT_NO_THROW((void)serialized.get<vt::State>());
}
TEST(VdaState, InvalidPlaceholderCannotPreventPublishingStateBody) {
  namespace vt=vda5050_core::types;
  nlohmann::json payload=vt::State{};
  payload["timestamp"]="2026-01-01T00:00:00Z";
  EXPECT_NO_THROW((void)fleet::state_from_payload(payload));
}

#include "linorobot2_fleet_bridge/vda_instant_actions.hpp"
namespace {
nlohmann::json numeric_init_position() {
  return {{"headerId",1},{"timestamp","2026-11-08T12:00:00.000Z"},
    {"version","2.0.0"},{"manufacturer","AGVLab"},{"serialNumber","LAB-01"},
    {"actions",nlohmann::json::array({{{"actionId","init-1"},{"actionType","initPosition"},
      {"blockingType","HARD"},{"actionParameters",nlohmann::json::array({
        {{"key","x"},{"value",-4.0}},{{"key","y"},{"value",0}},
        {{"key","theta"},{"value",0.0}},{{"key","mapId"},{"value","fleet-lab-r3a"}},
        {{"key","lastNodeId"},{"value","L-STAGE"}}})}}})}};
}
}
TEST(VdaInstantActions, NumericInitPositionValidatesWithoutChangingWireValues) {
  const auto raw=numeric_init_position();
  const auto original=raw.dump();
  EXPECT_NO_THROW(fleet::validate_instant_actions(raw));
  EXPECT_EQ(raw.dump(),original);
  EXPECT_TRUE(raw["actions"][0]["actionParameters"][0]["value"].is_number());
}
TEST(VdaInstantActions, InvalidCoordinateAndEnvelopeAreStillRejected) {
  auto raw=numeric_init_position();
  raw["actions"][0]["actionParameters"][0]["value"]=true;
  EXPECT_ANY_THROW(fleet::validate_instant_actions(raw));
  raw=numeric_init_position();raw["actions"][0].erase("actionId");
  EXPECT_ANY_THROW(fleet::validate_instant_actions(raw));
}
