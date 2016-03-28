#include <cfloat>

#include "weak_classifier.h"

WeakClassifier::WeakClassifier() :
  feature (Feature(0)),
  error (FLT_MAX)
{}

WeakClassifier::WeakClassifier(const Feature& feature, float threshold, int polarity, float error) :
  feature (feature),
  threshold (threshold),
  polarity (polarity),
  error (error)
{}

bool WeakClassifier::operator<(const WeakClassifier& right) const {
  return this->error < right.error;
}

std::ostream& operator<<(std::ostream& os, const WeakClassifier& classifier) {
  return os <<
    "feature: " << std::endl << classifier.feature <<
    "threshold: " << classifier.threshold << std::endl <<
    "polarity: " << classifier.polarity <<
    "error: " << classifier.error;
}
