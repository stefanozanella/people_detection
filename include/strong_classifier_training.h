#ifndef STRONG_CLASSIFIER_TRAINING_H
#define STRONG_CLASSIFIER_TRAINING_H

#include <vector>

#include "strong_classifier.h"
#include "training_sample.h"
#include "weak_classifier.h"

using std::vector;

class StrongClassifierTraining {
  vector<TrainingSample>& samples;
  const vector<Feature>& features;
  StrongClassifier& strong;

  void normalize_weights(vector<TrainingSample>& samples);
  WeakClassifier optimal_classifier(vector<TrainingSample>& samples, const vector<Feature>& features);
  WeakClassifier optimal_classifier_for_feature(const Feature& feature, vector<TrainingSample>& samples);
  void update_weights(vector<TrainingSample>& samples, const WeakClassifier& classifier);

  public:

  StrongClassifierTraining(
    // TODO Should be const
    vector<TrainingSample>& samples,
    const vector<Feature>& features,
    StrongClassifier& strong
  );
  void trainWeakClassifier();
};

#endif
