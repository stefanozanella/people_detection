#ifndef LOAD_TRAINED_DETECTOR_H
#define LOAD_TRAINED_DETECTOR_H

#include <string>

#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "strong_classifier.h"
#include "weak_classifier.h"
#include "feature.h"
#include "rect.h"

using std::string;
using boost::filesystem::is_regular_file;

bool load_trained_detector(const string& filename, StrongClassifier& classifier) {
  if (!is_regular_file(filename)) {
    return false;
  }

  YAML::Node params = YAML::LoadFile(filename);

  for (
    YAML::const_iterator weak = params["weak_classifiers"].begin();
    weak != params["weak_classifiers"].end();
    weak++
  ) {
    Feature feature ((*weak)["feature"]["base_size"].as<int>());

    for (
      YAML::const_iterator rect = (*weak)["feature"]["rectangles"].begin();
      rect != (*weak)["feature"]["rectangles"].end();
      rect++
    ) {
      feature << Rect(
        (*rect)["x"].as<int>(),
        (*rect)["y"].as<int>(),
        (*rect)["width"].as<int>(),
        (*rect)["height"].as<int>(),
        (*rect)["multiplier"].as<int>()
      );
    }

    classifier << WeakClassifier(
      feature,
      (*weak)["threshold"].as<float>(),
      (*weak)["polarity"].as<int>(),
      (*weak)["error"].as<float>()
    );
  }

  return true;
}

#endif
