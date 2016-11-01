#ifndef WEAK_CLASSIFIER_H
#define WEAK_CLASSIFIER_H

#include <iostream>
#include "feature.h"
#include "sample.h"
#include "storage.h"

class WeakClassifier {
  public:

  Feature feature;
  float threshold;
  int polarity;
  float error;

  WeakClassifier();
  WeakClassifier(const Feature& feature, float threshold, int polarity, float error);
  bool classify(const Sample& sample) const;
  float error_weight_factor() const;
  float classification_factor() const;
  bool operator<(const WeakClassifier& right) const;
  void save(Storage& storage) const;
  friend std::ostream& operator<<(std::ostream& os, const WeakClassifier& classifier);
};

#endif
