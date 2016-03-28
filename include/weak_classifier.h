#ifndef WEAK_CLASSIFIER_H
#define WEAK_CLASSIFIER_H

#include <iostream>
#include "feature.h"

class WeakClassifier {
  Feature feature;
  float threshold;
  int polarity;
  float error;

  public:

  WeakClassifier();
  WeakClassifier(const Feature& feature, float threshold, int polarity, float error);
  bool operator<(const WeakClassifier& right) const;
  friend std::ostream& operator<<(std::ostream& os, const WeakClassifier& classifier);
};

#endif
