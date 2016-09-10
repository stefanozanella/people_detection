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
  void push_back(const WeakClassifier& weak);
  bool is_face(const Sample& sample) const;
  void force_detection(const Sample& sample); // TODO Remove
  void save(Storage& storage) const;
  void adjust_threshold(const float adjustment);
};

#endif
