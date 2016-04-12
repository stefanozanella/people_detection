#include "strong_classifier.h"

StrongClassifier::StrongClassifier() :
  threshold (0)
{}

StrongClassifier& StrongClassifier::operator<<(const WeakClassifier& classifier) {
  classifiers.push_back(classifier);
  threshold += classifier.classification_factor();

  return *this;
}

bool StrongClassifier::classify(const TrainingSample& sample) const {
  float classification_value = 0;

  for (vector<WeakClassifier>::const_iterator classifier = classifiers.begin(); classifier != classifiers.end(); classifier++) {
    classification_value += classifier->classification_factor() * classifier->classify(sample);
  }

  return classification_value >= 0.5 * threshold;
}
