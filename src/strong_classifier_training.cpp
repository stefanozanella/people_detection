#include "strong_classifier_training.h"
#include "feature_value.h"

#include <algorithm>
#include <limits>
#include <cmath>

using std::max;
using std::min;
using std::sort;
using std::numeric_limits;
using std::abs;

StrongClassifierTraining::StrongClassifierTraining(vector<TrainingSample>& samples, const vector<Feature>& features, StrongClassifier& strong) :
  samples (samples),
  features (features),
  strong (strong)
{
  initialize_weights(samples);
}

int StrongClassifierTraining::max_false_negatives_for_detection_rate(const float detection_rate) {
  int positive_count = 0;
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive) {
      positive_count++;
    }
  }

  return (int) (positive_count * (1 - detection_rate));
}

// TODO: Camel case?
void StrongClassifierTraining::trainWeakClassifier(const float min_detection_rate) {
  int max_false_negatives = max_false_negatives_for_detection_rate(min_detection_rate);

  normalize_weights(samples);
  last_trained_classifier = optimal_classifier(samples, features, max_false_negatives);
  update_weights(samples, last_trained_classifier);

  strong.push_back(last_trained_classifier);
}

void StrongClassifierTraining::adjust_threshold(const float adjustment) {
  reset_weights(samples, last_trained_classifier);
  strong.pop_back();
  last_trained_classifier.threshold += last_trained_classifier.polarity * abs(last_trained_classifier.threshold) * adjustment;
  last_trained_classifier.error = compute_classifier_error(samples, last_trained_classifier);
  strong.push_back(last_trained_classifier);
  update_weights(samples, last_trained_classifier);
}

void StrongClassifierTraining::initialize_weights(vector<TrainingSample>& samples) {
  float positive_cumulative_weight = 0, negative_cumulative_weight = 0;

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive)
      positive_cumulative_weight += sample->weight;
    else
      negative_cumulative_weight += sample->weight;
  }

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    sample->weight /= sample->isPositive ? positive_cumulative_weight : negative_cumulative_weight;
  }
}

void StrongClassifierTraining::normalize_weights(vector<TrainingSample>& samples) {
  float total_weight = 0;

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    total_weight += sample->weight;
  }

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    sample->weight /= total_weight;
  }
}

WeakClassifier StrongClassifierTraining::optimal_classifier(vector<TrainingSample>& samples, const vector<Feature>& features, const int max_false_negatives) {
  WeakClassifier optimal_classifier;

  for (int k = 0; k < features.size(); k += 1) {
    optimal_classifier = min(
      optimal_classifier,
      optimal_classifier_for_feature(features.at(k), samples, max_false_negatives)
    );
  }

  return optimal_classifier;
}

WeakClassifier StrongClassifierTraining::optimal_classifier_for_feature(const Feature& feature, vector<TrainingSample>& samples, const int max_false_negatives) {
  vector<FeatureValue> feature_values;

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    feature_values.push_back(FeatureValue(feature, *sample));
  }

  sort(feature_values.begin(), feature_values.end());

  float total_positive_weight_sum = 0, total_negative_weight_sum = 0, positive_below = 0, negative_below = 0;
  int total_positive_samples = 0, positive_samples_below = 0;

  for (vector<FeatureValue>::iterator feature_value = feature_values.begin(); feature_value != feature_values.end(); feature_value++) {
    // TODO Feature envy? Maybe add a FeatureValue::update_weight_sum(float, float) ?
    if (feature_value->sample.isPositive) {
      total_positive_weight_sum += feature_value->sample.weight;
      total_positive_samples++;
    } else {
      total_negative_weight_sum += feature_value->sample.weight;
    }
  }

  float optimal_threshold, optimal_error = numeric_limits<float>::max();
  int optimal_polarity;
  float previous_feature_value = 0;

  for (vector<FeatureValue>::iterator feature_value = feature_values.begin(); feature_value != feature_values.end(); feature_value++) {
    float error_when_polarity_is_negative = positive_below + total_negative_weight_sum - negative_below;
    float error_when_polarity_is_positive = negative_below + total_positive_weight_sum - positive_below;
    int false_negatives_when_polarity_is_negative = positive_samples_below;
    int false_negatives_when_polarity_is_positive = total_positive_samples - positive_samples_below;

    float optimal_error_for_this_sample;
    int optimal_polarity_for_this_sample;
    float optimal_threshold_for_this_sample;

    if (error_when_polarity_is_positive <= error_when_polarity_is_negative && false_negatives_when_polarity_is_positive <= max_false_negatives) {
      optimal_error_for_this_sample = error_when_polarity_is_positive;
      optimal_polarity_for_this_sample = 1;
      optimal_threshold_for_this_sample = feature_value->value;
    }
    else if (false_negatives_when_polarity_is_negative <= max_false_negatives) {
      optimal_error_for_this_sample = error_when_polarity_is_negative;
      optimal_polarity_for_this_sample = -1;
      optimal_threshold_for_this_sample = feature_value->value;
    }

    if (optimal_error_for_this_sample < optimal_error) {
      optimal_error = optimal_error_for_this_sample;
      optimal_polarity = optimal_polarity_for_this_sample;
      optimal_threshold = optimal_threshold_for_this_sample;
    }

    // NOTE: There are cases where a feature completely fails to discriminate
    // one example from the other. One example of this is when the rectangles
    // of a base feature are scaled down to sub-pixel sizes when applying the
    // feature to a specific sample, thus leading to positive and negative
    // areas eliding themselves. In such a case the risk is that the sorting
    // algorithm outputs the feature values, which are all the same, in a way
    // so that at a specific randomly-picked position in the list a feature
    // has a computed error lower than the best non-trivial feature just
    // because of the position it holds in the randomly-sorted list. When this
    // happens, all subsequent training is spoiled. For this reason, we
    // increase the counters of S+ and S- only if the values of the feature
    // is strictly greater than the previous one.
    //
    // Another way to look at this is to consider that if the sorting
    // algorithm was to sort differently samples with the same feature value,
    // two samples with the same feature value could end up in different order,
    // resulting in a different count of what's below and what's above a
    // certain threshold.

    // TODO Feature envy? Maybe review and use FeatureValue comparison +
    // FeatureValue::update_weight_sum(float, float)?
    if (feature_value->value > previous_feature_value)
      if (feature_value->sample.isPositive) {
        positive_below += feature_value->sample.weight;
        positive_samples_below++;
      } else {
        negative_below += feature_value->sample.weight;
      }
    previous_feature_value = feature_value->value;
  }

  return WeakClassifier(feature, optimal_threshold, optimal_polarity, optimal_error);
}

void StrongClassifierTraining::update_weights(vector<TrainingSample>& samples, const WeakClassifier& classifier) {
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    // TODO Cleanup
    if (sample->isPositive && !classifier.classify(*sample)) {
      //cout << "Ouch! False negative" << endl;
    } else if (!sample->isPositive && classifier.classify(*sample)) {
      //cout << "Meh! False positive" << endl;
    }
    else {
      sample->weight *= classifier.error_weight_factor();
    }
  }
}

void StrongClassifierTraining::reset_weights(vector<TrainingSample>& samples, const WeakClassifier& classifier) {
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    // TODO Cleanup
    if (sample->isPositive && !classifier.classify(*sample)) {
      //cout << "Ouch! False negative" << endl;
    } else if (!sample->isPositive && classifier.classify(*sample)) {
      //cout << "Meh! False positive" << endl;
    }
    else {
      sample->weight /= classifier.error_weight_factor();
    }
  }
}

float StrongClassifierTraining::compute_classifier_error(vector<TrainingSample>& samples, const WeakClassifier& classifier) {
  float classifier_error = 0.0f;
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive != classifier.classify(*sample)) {
      classifier_error += sample->weight;
    }
  }
  return classifier_error;
}

void StrongClassifierTraining::discard_last_trained_classifier() {
  strong.pop_back();
}
