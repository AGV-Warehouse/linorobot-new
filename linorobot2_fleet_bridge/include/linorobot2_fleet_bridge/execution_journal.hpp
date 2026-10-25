#pragma once
#include <string>
#include <optional>
#include <sqlite3.h>
namespace fleet {
class Journal {
public:
  explicit Journal(const std::string& path);
  ~Journal();
  Journal(const Journal&)=delete;
  Journal& operator=(const Journal&)=delete;
  std::optional<std::string> get(const std::string& key) const;
  void put(const std::string& key,const std::string& value);
private:
  sqlite3* db_=nullptr;
};
}
