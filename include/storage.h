#ifndef STORAGE_H
#define STORAGE_H

#include <string>
#include "yaml-cpp/yaml.h"

class Storage {
  YAML::Node yaml;

  public:

  Storage();

  template <typename Value> void add(const std::string key, const Value& value) {
    yaml[key] = value;
  }

  void add(const std::string key, const Storage& value) {
    yaml[key] = value.yaml;
  }

  void push_back(const std::string key, const Storage& node);
  void persist(const std::string filename);
};

#endif
