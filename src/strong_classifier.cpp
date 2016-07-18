#include "strong_classifier.h"

StrongClassifier::StrongClassifier() :
  threshold (0)
{}

StrongClassifier& StrongClassifier::operator<<(const WeakClassifier& classifier) {
  classifiers.push_back(classifier);
  threshold += 0.5 * classifier.classification_factor();

  return *this;
}

float StrongClassifier::classification_value(const Sample& sample) const {
  float classification_value = 0;

  for (vector<WeakClassifier>::const_iterator classifier = classifiers.begin(); classifier != classifiers.end(); classifier++) {
    classification_value += classifier->classification_factor() * classifier->classify(sample);
  }

  return classification_value;
}

bool StrongClassifier::is_face(const Sample& sample) const {
  return classification_value(sample) <= threshold;
}

void StrongClassifier::force_detection(const Sample& sample) {
  threshold = classification_value(sample);
}

void StrongClassifier::save(Storage& storage) const {
  storage.add("threshold", threshold);
  for (vector<WeakClassifier>::const_iterator classifier = classifiers.begin(); classifier != classifiers.end(); classifier++) {
    Storage weak_classifier;
    classifier->save(weak_classifier);
    storage.push_back("weak_classifiers", weak_classifier);
  }
}
