#ifndef STRONG_CLASSIFIER_H
#define STRONG_CLASSIFIER_H

#include <vector>

#include "training_sample.h"
#include "weak_classifier.h"

using std::vector;

class StrongClassifier {
  vector<WeakClassifier> classifiers;
  float threshold;

  public:

  StrongClassifier();
  StrongClassifier& operator<<(const WeakClassifier& classifier);
  bool classify(const TrainingSample& sample) const;
};

#endif
