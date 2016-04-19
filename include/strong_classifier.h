#ifndef STRONG_CLASSIFIER_H
#define STRONG_CLASSIFIER_H

#include <vector>

#include "training_sample.h"
#include "weak_classifier.h"
#include "storage.h"

using std::vector;

class StrongClassifier {
  vector<WeakClassifier> classifiers;
  float threshold;

  float classification_value(const TrainingSample& sample) const;

  public:

  StrongClassifier();
  StrongClassifier& operator<<(const WeakClassifier& classifier);
  bool classify(const TrainingSample& sample) const;
  void force_detection(const TrainingSample& sample);
  void save(Storage& storage) const;
};

#endif
