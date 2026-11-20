#include <gtest/gtest.h>
#include "linorobot2_fleet_bridge/node_actions.hpp"
#include "linorobot2_fleet_bridge/vda_actions.hpp"
using namespace fleet;
namespace {
Json station_order(const std::string& id,const std::string& type) {
  auto o=Json::parse(R"({"orderId":"o1","orderUpdateId":0,"nodes":[
    {"nodeId":"a","sequenceId":0,"released":true,"nodePosition":{"x":0,"y":0,"mapId":"map"},"actions":[]},
    {"nodeId":"b","sequenceId":2,"released":true,"nodePosition":{"x":2,"y":0,"mapId":"map"},"actions":[]}],
    "edges":[{"edgeId":"ab","sequenceId":1,"released":true,"startNodeId":"a","endNodeId":"b","actions":[]}]})");
  o["orderId"]=id;
  o["nodes"][1]["actions"].push_back({{"actionId","Order_destination_action_1"},{"actionType",type},{"blockingType","HARD"},{"actionParameters",Json::array()}});
  return o;
}
void set_status(OrderBook& b,const std::string& status) {
  for(auto& a:b.actions) if(a.at("actionId")=="Order_destination_action_1") a["actionStatus"]=status;
}
}
TEST(NodeActions, KnownTypesAreTyped) {
  EXPECT_EQ(action_kind("waitForTrigger"),ActionKind::WaitForTrigger);
  EXPECT_EQ(action_kind("pick"),ActionKind::Pick);
  EXPECT_EQ(action_kind("drop"),ActionKind::Drop);
  EXPECT_EQ(action_kind("startCharging"),ActionKind::StartCharging);
  EXPECT_EQ(action_kind("lift"),ActionKind::Unsupported);
  EXPECT_TRUE(needs_operator(ActionKind::Pick));
  EXPECT_FALSE(needs_operator(ActionKind::StartCharging));
}
TEST(NodeActions, RejectUnsupportedDuplicateAndEdgeActions) {
  Journal j(":memory:");OrderBook b(j);b.cancel();
  auto lift=station_order("o1","lift");EXPECT_THROW(b.accept(lift,"map"),std::invalid_argument);
  auto twice=station_order("o1","pick");twice["nodes"][1]["actions"].push_back(twice["nodes"][1]["actions"][0]);
  EXPECT_THROW(b.accept(twice,"map"),std::invalid_argument);
  auto edge=station_order("o1","pick");edge["edges"][0]["actions"].push_back(edge["nodes"][1]["actions"][0]);
  EXPECT_THROW(b.accept(edge,"map"),std::invalid_argument);
}
TEST(NodeActions, ReportedWaitingAndBlockOnlyAfterArrival) {
  Journal j(":memory:");OrderBook b(j);b.cancel();
  b.accept(station_order("o1","waitForTrigger"),"map");
  ASSERT_EQ(b.actions.size(),1u);EXPECT_EQ(b.actions[0]["actionStatus"],"WAITING");
  b.complete();EXPECT_TRUE(b.pending_node_actions().empty());  // at start node a
  b.complete();ASSERT_EQ(b.pending_node_actions().size(),1u);  // arrived at b
  set_status(b,"RUNNING");EXPECT_EQ(b.pending_node_actions().size(),1u);
  set_status(b,"FINISHED");EXPECT_TRUE(b.pending_node_actions().empty());
}
TEST(NodeActions, NextOrderWaitsForStationAndForgetsReusedIds) {
  Journal j(":memory:");OrderBook b(j);b.cancel();
  b.accept(station_order("o1","pick"),"map");b.complete();b.complete();
  auto next=station_order("o2","drop");
  EXPECT_THROW(b.accept(next,"map"),std::invalid_argument);
  set_status(b,"FINISHED");
  EXPECT_NO_THROW(b.accept(next,"map"));
  ASSERT_EQ(b.actions.size(),1u);
  EXPECT_EQ(b.actions[0]["actionType"],"drop");EXPECT_EQ(b.actions[0]["actionStatus"],"WAITING");
}
TEST(NodeActions, ReusedIdInLaterUpdateOfNextOrderStartsWaiting) {
  // openTCS numbers destination actions by route index: the drop of the next
  // drive order can reuse the pick's ID and arrive only in a later update.
  Journal j(":memory:");OrderBook b(j);b.cancel();
  b.accept(station_order("o1","pick"),"map");b.complete();b.complete();set_status(b,"FINISHED");
  auto next=station_order("o2","drop");
  auto first=next;first["nodes"][1]["released"]=false;first["edges"][0]["released"]=false;
  first["nodes"][1]["actions"]=Json::array();
  b.accept(first,"map");
  auto update=next;update["orderUpdateId"]=1;
  b.accept(update,"map");
  ASSERT_EQ(b.actions.size(),1u);
  EXPECT_EQ(b.actions[0]["actionType"],"drop");EXPECT_EQ(b.actions[0]["actionStatus"],"WAITING");
}
