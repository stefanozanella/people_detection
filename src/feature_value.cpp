#include "feature_value.h"

FeatureValue::FeatureValue(const Feature& feature, const TrainingSample& sample) :
  sample (sample)
{
  value = feature.apply(sample);
}

bool FeatureValue::operator<(const FeatureValue &right) const {
  return this->value < right.value;
}
