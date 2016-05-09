#ifndef STRONG_CLASSIFIER_H
#define STRONG_CLASSIFIER_H

#include <vector>

#include "sample.h"
#include "weak_classifier.h"
#include "storage.h"

using std::vector;

class StrongClassifier {
  vector<WeakClassifier> classifiers;
  float threshold;

  float classification_value(const Sample& sample) const;

  public:

  StrongClassifier();
  StrongClassifier& operator<<(const WeakClassifier& classifier);
  bool is_face(const Sample& sample) const;
  void force_detection(const Sample& sample);
  void save(Storage& storage) const;
};

#endif
