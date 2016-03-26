#include "weak_classifier.h"

WeakClassifier::WeakClassifier() :
  feature (Feature(0))
{}

WeakClassifier::WeakClassifier(const Feature& feature, float threshold, int polarity) :
  feature (feature),
  threshold (threshold),
  polarity (polarity)
{}

std::ostream& operator<<(std::ostream& os, const WeakClassifier& classifier) {
  return os <<
    "feature: " << std::endl << classifier.feature <<
    "threshold: " << classifier.threshold << std::endl <<
    "polarity: " << classifier.polarity;
}
