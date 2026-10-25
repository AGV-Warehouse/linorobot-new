#include <gtest/gtest.h>
#include <filesystem>
#include <chrono>
#include "linorobot2_fleet_bridge/vda_actions.hpp"
using namespace fleet;
TEST(Recovery, RestartNeverRestoresMotion) {
  Journal j(":memory:"); j.put("execution",R"({"order":{"orderId":"pending"},"index":0,"actions":[],"lastNodeId":"a","lastNodeSequenceId":0})");
  OrderBook b(j);EXPECT_TRUE(b.paused);EXPECT_TRUE(b.blocked);b.cancel();EXPECT_FALSE(b.blocked);EXPECT_TRUE(b.paused);EXPECT_TRUE(j.get("cancelled:pending"));
}
TEST(Recovery, JournalOverwritesAtomically) {Journal j(":memory:");EXPECT_FALSE(j.get("k"));j.put("k","RUNNING");j.put("k","FINISHED");EXPECT_EQ(j.get("k").value(),"FINISHED");}
TEST(Recovery, RecordsSurviveDatabaseReopen) {
  const auto unique=std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path=std::filesystem::temp_directory_path()/("fleet-journal-"+std::to_string(unique)+".sqlite");
  {Journal j(path.string());j.put("action:1","FINISHED");}
  {Journal j(path.string());EXPECT_EQ(j.get("action:1").value(),"FINISHED");}
  std::filesystem::remove(path);
}
