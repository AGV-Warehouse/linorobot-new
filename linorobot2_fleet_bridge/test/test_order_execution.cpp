#include <gtest/gtest.h>
#include "linorobot2_fleet_bridge/vda_actions.hpp"
#include "linorobot2_fleet_bridge/route_guard.hpp"
using namespace fleet;
Json sample() {return Json::parse(R"({"orderId":"o1","orderUpdateId":0,"nodes":[{"nodeId":"a","sequenceId":0,"released":true,"nodePosition":{"x":0,"y":0,"mapId":"map"},"actions":[]},{"nodeId":"b","sequenceId":2,"released":false,"nodePosition":{"x":2,"y":0,"mapId":"map"},"actions":[]}],"edges":[{"edgeId":"ab","sequenceId":1,"released":false,"startNodeId":"a","endNodeId":"b","actions":[]}]})");}
TEST(Orders, HorizonCannotExecute) {Journal j(":memory:");OrderBook b(j);b.accept(sample(),"map");b.complete();EXPECT_TRUE(b.next().is_null());}
TEST(Orders, RejectReusedVersion) {Journal j(":memory:");OrderBook b(j);auto o=sample();b.accept(o,"map");o["nodes"][1]["nodePosition"]["x"]=3;EXPECT_THROW(b.accept(o,"map"),std::invalid_argument);}
TEST(Orders, RejectActionsAndWrongMap) {
  Journal j(":memory:");OrderBook b(j);auto o=sample();
  EXPECT_THROW(b.accept(o,"other"),std::invalid_argument);
  o["nodes"][0]["actions"].push_back({{"actionType","pick"}});
  EXPECT_THROW(b.accept(o,"map"),std::invalid_argument);
}
TEST(Orders, ExtendBaseBeforeReachingDecisionPoint) {
  Journal j(":memory:");OrderBook b(j);auto o=sample();
  o["nodes"][1]["released"]=true;o["edges"][0]["released"]=true;
  b.accept(o,"map");b.complete();
  auto update=o;update["orderUpdateId"]=1;
  update["nodes"]=Json::array({o["nodes"][1],{{"nodeId","c"},{"sequenceId",4},{"released",true},{"nodePosition",{{"x",4},{"y",0},{"mapId","map"}}},{"actions",Json::array()}}});
  update["edges"]=Json::array({{{"edgeId","bc"},{"sequenceId",3},{"released",true},{"startNodeId","b"},{"endNodeId","c"},{"actions",Json::array()}}});
  b.accept(update,"map");EXPECT_EQ(b.next()["nodeId"],"b");
  b.complete();EXPECT_EQ(b.next()["nodeId"],"c");
  EXPECT_NO_THROW(b.accept(update,"map"));
}
TEST(Route, RejectShortcutOutsideAuthorizedCapsule) {RouteGuard g(0.15);g.authorize({0,0,0},{2,0,0});EXPECT_TRUE(g.accepts({{0,0,0},{1,0.1,0},{2,0,0}}));EXPECT_FALSE(g.accepts({{0,0,0},{1,0.3,0},{2,0,0}}));g.revoke();EXPECT_FALSE(g.contains({1,0,0}));}
TEST(Orders, OldCompletedOrderNeverRunsAgain) {
  Journal j(":memory:");OrderBook b(j);auto o=sample();
  o["nodes"][1]["released"]=true;o["edges"][0]["released"]=true;
  b.accept(o,"map");b.complete();b.complete();
  auto second=o;second["orderId"]="second";b.accept(second,"map");b.complete();b.complete();
  EXPECT_THROW(b.accept(o,"map"),std::invalid_argument);
}
