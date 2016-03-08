#ifndef FEATURE_VALUE_H
#define FEATURE_VALUE_H

#include "feature.h"
#include "training_sample.h"

class FeatureValue {
  TrainingSample sample;
  float value;

  public:
  FeatureValue(const Feature& feature, const TrainingSample& sample);
  bool operator<(const FeatureValue& right) const;
};

#endif
