#include "storage.h"
#include <fstream>

using std::string;
using std::ofstream;

Storage::Storage() : yaml (YAML::NodeType::Map) {}

void Storage::push_back(const string key, const Storage& node) {
  yaml[key].push_back(node.yaml);
}

void Storage::persist(const string filename) {
  ofstream out (filename.c_str());
  out << yaml;
  out.close();
}
