#include "linorobot2_fleet_bridge/execution_journal.hpp"
#include <filesystem>
#include <stdexcept>
namespace fleet {
Journal::Journal(const std::string& path) {
  if (path!=":memory:" && std::filesystem::path(path).has_parent_path())
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  if (sqlite3_open(path.c_str(),&db_)!=SQLITE_OK) throw std::runtime_error("Cannot open journal");
  sqlite3_busy_timeout(db_,2000);
  if (sqlite3_exec(db_,"PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL; CREATE TABLE IF NOT EXISTS journal (key TEXT PRIMARY KEY,value TEXT NOT NULL);",nullptr,nullptr,nullptr)!=SQLITE_OK)
    throw std::runtime_error("Cannot initialize journal");
}
Journal::~Journal() { if(db_) sqlite3_close(db_); }
std::optional<std::string> Journal::get(const std::string& key) const {
  sqlite3_stmt* s=nullptr;
  if(sqlite3_prepare_v2(db_,"SELECT value FROM journal WHERE key=?",-1,&s,nullptr)!=SQLITE_OK) throw std::runtime_error("Journal read failed");
  sqlite3_bind_text(s,1,key.c_str(),-1,SQLITE_TRANSIENT);
  const int result=sqlite3_step(s);
  std::optional<std::string> value;
  if(result==SQLITE_ROW) value=reinterpret_cast<const char*>(sqlite3_column_text(s,0));
  sqlite3_finalize(s);
  if(result!=SQLITE_ROW && result!=SQLITE_DONE) throw std::runtime_error("Journal read failed");
  return value;
}
void Journal::put(const std::string& key,const std::string& value) {
  sqlite3_stmt* s=nullptr;
  if(sqlite3_prepare_v2(db_,"INSERT INTO journal(key,value) VALUES(?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",-1,&s,nullptr)!=SQLITE_OK) throw std::runtime_error("Journal write failed");
  sqlite3_bind_text(s,1,key.c_str(),-1,SQLITE_TRANSIENT);
  sqlite3_bind_text(s,2,value.c_str(),-1,SQLITE_TRANSIENT);
  const int result=sqlite3_step(s); sqlite3_finalize(s);
  if(result!=SQLITE_DONE) throw std::runtime_error("Journal commit failed");
}
}
