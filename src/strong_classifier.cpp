#include "strong_classifier.h"

StrongClassifier::StrongClassifier() :
  threshold (0)
{}

void StrongClassifier::push_back(const WeakClassifier& classifier) {
  classifiers.push_back(classifier);
  threshold += 0.5 * classifier.classification_factor();
}

void StrongClassifier::pop_back() {
  threshold -= 0.5 * classifiers.back().classification_factor();
  classifiers.pop_back();
}

float StrongClassifier::classification_value(const Sample& sample) const {
  float classification_value = 0;

  for (vector<WeakClassifier>::const_iterator classifier = classifiers.begin(); classifier != classifiers.end(); classifier++) {
    classification_value += classifier->classification_factor() * classifier->classify(sample);
  }

  return classification_value;
}

bool StrongClassifier::is_face(const Sample& sample) const {
  return classification_value(sample) >= threshold;
}

void StrongClassifier::save(Storage& storage) const {
  storage.add("threshold", threshold);
  for (vector<WeakClassifier>::const_iterator classifier = classifiers.begin(); classifier != classifiers.end(); classifier++) {
    Storage weak_classifier;
    classifier->save(weak_classifier);
    storage.push_back("weak_classifiers", weak_classifier);
  }
}
