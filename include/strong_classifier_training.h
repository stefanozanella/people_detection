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
  WeakClassifier last_trained_classifier;

  void initialize_weights(vector<TrainingSample>& samples);
  void normalize_weights(vector<TrainingSample>& samples);
  WeakClassifier optimal_classifier(vector<TrainingSample>& samples, const vector<Feature>& features, const int max_false_negatives);
  WeakClassifier optimal_classifier_for_feature(const Feature& feature, vector<TrainingSample>& samples, const int max_false_negatives);
  void update_weights(vector<TrainingSample>& samples, const WeakClassifier& classifier);
  void reset_weights(vector<TrainingSample>& samples, const WeakClassifier& classifier);
  float compute_classifier_error(vector<TrainingSample>& samples, const WeakClassifier& classifier);
  int max_false_negatives_for_detection_rate(const float detection_rate);

  public:

  StrongClassifierTraining(
    vector<TrainingSample>& samples,
    const vector<Feature>& features,
    StrongClassifier& strong
  );
  void trainWeakClassifier(const float min_detection_rate);
  void adjust_threshold(const float adjustment);
  void discard_last_trained_classifier();
};

#endif
