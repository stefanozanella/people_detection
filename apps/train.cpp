/**
 * GOAL:
 * [x] load samples
 * [x] calculate samples' integral image
 * [x] samples normalization
 * [x] generate features
 * [x] verify generated features and feature calculation
 * [x] select first weak classifier
 *   [x] normalize the weights
 *   [x] for each feature
 *     [x] compute feature value for all samples
 *     [x] sort feature values
 *     [x] compute optimal threshold and polarity
 *     TODO Refactor and document
 *   [x] update the weights
 * [x] build strong classifier from weak classifiers
 * [x] store partial training results
 * TODO extract storing logic into separate classes?
 * [x] load eventual partial training results
 * [x] skip training partially if results loaded from disk
 * [x] threshold lowering for detection rate increase
 * [ ] **validate approach**
 *   [?] try to reproduce old integral image algorithm where the square sum was
 *   computed at once and check if results change/improve.
 * [ ] expand training set
 *   [ ] add more positive samples
 *   [ ] add more negative samples
 *     [?] extract them from detections made by the first stage on the
 *     validation set?
 */
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include "training_sample.h"
#include "rect.h"
#include "feature.h"
#include "feature_value.h"
#include "weak_classifier.h"
#include "strong_classifier.h"
#include "storage.h"
#include "load_trained_detector.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using pcl::io::loadPCDFile;
using std::max;
using std::min;
using std::sort;

void generate_features(vector<Feature>& features, int window_size) {
  for (uint32_t x = 0; x < window_size; x++) {
    for (uint32_t y = 0; y < window_size; y++) {
      // generate horizontal haar2
      for (uint32_t dx = 1; 2*dx < window_size - x; dx++) {
        for (uint32_t dy = 1; dy < window_size - y; dy++) {
          features.push_back(
            Feature(window_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1)
          );
        }
      }

      // generate vertical haar2
      for (int dx = 1; dx < window_size - x; dx++) {
        for (int dy = 1; 2*dy < window_size - y; dy++) {
          features.push_back(
            Feature(window_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x, y + dy, dx, dy, 1)
          );
        }
      }

      // generate horizontal haar3
      for (int dx = 1; 3*dx < window_size - x; dx++) {
        for (int dy = 1; dy < window_size - y; dy++) {
          features.push_back(
            Feature(window_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1) <<
            Rect(x + 2*dx, y, dx, dy, -1)
          );
        }
      }

      // generate vertical haar3
      for (int dx = 1; dx < window_size - x; dx++) {
        for (int dy = 1; 3*dy < window_size - y; dy++) {
          features.push_back(
            Feature(window_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x, y + dy, dx, dy, 1) <<
            Rect(x, y + 2*dy, dx, dy, -1)
          );
        }
      }

      // generate haar4
      for (int dx = 1; 4*dx < window_size - x; dx++) {
        for (int dy = 1; 4*dy < window_size - y; dy++) {
          features.push_back(
            Feature(window_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1) <<
            Rect(x, y + dy, dx, dy, 1) <<
            Rect(x + dx, y + dy, dx, dy, -1)
          );
        }
      }
    }
  }
}

int load_samples(vector<TrainingSample>& samples, const string& positive_samples_dir, const string& negative_samples_dir) {
  directory_iterator positive_sample ((path(positive_samples_dir)));
  directory_iterator negative_sample ((path(negative_samples_dir)));
  directory_iterator no_more_samples;

  for (; positive_sample != no_more_samples; positive_sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(positive_sample->path().string(), *(sample_cloud)) == -1) {
      cout << "Couldn't load cloud file " << positive_sample->path() << endl;
      return -1;
    }

    samples.push_back(TrainingSample(sample_cloud, true));
  }

  for (; negative_sample != no_more_samples; negative_sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(negative_sample->path().string(), *(sample_cloud)) == -1) {
      cout << "Couldn't load cloud file " << negative_sample->path() << endl;
      return -1;
    }

    samples.push_back(TrainingSample(sample_cloud, false));
  }

  return 0;
}

/**
 * Invariants:
 * * samples are square in size (width == height)
 * * samples are organized clouds
 */
uint32_t find_biggest_window(const vector<TrainingSample>& samples) {
  uint32_t biggest_window_size = 0;
  for (vector<TrainingSample>::const_iterator sample = samples.begin(); sample != samples.end(); sample++) {
    biggest_window_size = max(biggest_window_size, sample->size);
  }

  return biggest_window_size;
}

void normalize_weights(vector<TrainingSample>& samples) {
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

WeakClassifier optimal_classifier_for_feature(Feature& feature, vector<TrainingSample>& samples) {
  vector<FeatureValue> feature_values;

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    feature_values.push_back(FeatureValue(feature, *sample));
  }

  sort(feature_values.begin(), feature_values.end());

  float total_positive_weight_sum = 0, total_negative_weight_sum = 0, positive_below = 0, negative_below = 0;

  for (vector<FeatureValue>::iterator feature_value = feature_values.begin(); feature_value != feature_values.end(); feature_value++) {
    // TODO Feature envy? Maybe add a FeatureValue::update_weight_sum(float, float) ?
    if (feature_value->sample.isPositive)
      total_positive_weight_sum += feature_value->sample.weight;
    else
      total_negative_weight_sum += feature_value->sample.weight;
  }

  float optimal_threshold, optimal_error = FLT_MAX;
  int optimal_polarity;
  float previous_feature_value = 0;

  for (vector<FeatureValue>::iterator feature_value = feature_values.begin(); feature_value != feature_values.end(); feature_value++) {
    float error_when_polarity_is_negative = positive_below + total_negative_weight_sum - negative_below;
    float error_when_polarity_is_positive = negative_below + total_positive_weight_sum - positive_below;

    float optimal_error_for_this_sample;
    int optimal_polarity_for_this_sample;
    float optimal_threshold_for_this_sample;

    if (error_when_polarity_is_positive <= error_when_polarity_is_negative) {
      optimal_error_for_this_sample = error_when_polarity_is_positive;
      optimal_polarity_for_this_sample = 1;
      optimal_threshold_for_this_sample = feature_value->value;
    }
    else {
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
      if (feature_value->sample.isPositive)
        positive_below += feature_value->sample.weight;
      else
        negative_below += feature_value->sample.weight;
    previous_feature_value = feature_value->value;
  }

  return WeakClassifier(feature, optimal_threshold, optimal_polarity, optimal_error);
}

WeakClassifier optimal_classifier(vector<TrainingSample>& samples, vector<Feature>& features) {
  WeakClassifier optimal_classifier;

  for (int k = 0; k < features.size(); k += 1) {
    optimal_classifier = min(
      optimal_classifier,
      optimal_classifier_for_feature(features.at(k), samples)
    );
  }

  return optimal_classifier;
}

void update_weights(vector<TrainingSample>& samples, const WeakClassifier& classifier) {
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive && !classifier.classify(*sample)) {
      cout << "Ouch! False negative" << endl;
    } else if (!sample->isPositive && classifier.classify(*sample)) {
      cout << "Meh! False positive" << endl;
    }
    else {
      sample->weight *= classifier.error_weight_factor();
    }
  }
}

void save_training_results(StrongClassifier& classifier, const string& file) {
  Storage storage;
  classifier.save(storage);
  storage.persist(file);
}

int main(int argc, char** argv) {
  vector<TrainingSample> samples;
  vector<Feature> features;

  if (load_samples(samples, string(argv[1]), string(argv[2])) < 0) {
    // TODO More informative error
    return -1;
  }

  generate_features(features, find_biggest_window(samples));
  cout << "Generated " << features.size() << " features" << endl;

  StrongClassifier strong;

  if (!load_trained_detector("face_detector.yml", strong)) {
    for (int t = 0; t < 2; t++) {
      normalize_weights(samples);
      WeakClassifier weak = optimal_classifier(samples, features);
      update_weights(samples, weak);

      strong << weak;

      cout << "Found best classifier: " << weak << endl;
    }

    save_training_results(strong, "face_detector.yml");
  }

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive && !strong.is_face(*sample)) {
      cout << "Ouch! False negative" << endl;
    }

    if (!sample->isPositive && strong.is_face(*sample)) {
      cout << "Meh! False positive" << endl;
    }
  }

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive && !strong.is_face(*sample)) {
      strong.force_detection(*sample);
    }
  }

  save_training_results(strong, "face_detector.yml");

  cout << "~~~~~~~~~~~" << endl;

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive && !strong.is_face(*sample)) {
      cout << "Ouch! False negative" << endl;
    }

    if (!sample->isPositive && strong.is_face(*sample)) {
      cout << "Meh! False positive" << endl;
    }
  }

  // DEFINITIONS:
  // false positive rate = sum(false positives) / sum(negative samples)
  // detection rate = sum(positive detection) / sum(positive samples)
  //
  // f = maximum acceptable false positive rate per layer
  // d = minimum acceptable detection rate per layer
  // F_target = target overall false positive rate
  // F_0 = 1.0 = current false positive rate
  // while F_i > F_target
  //  F_i = F_i-1
  //  n_i = 0
  //  while F_i > f * F_i-1
  //    n_i++
  //    train a strong classifier with n_i features
  //    evaluate F_i and D_i for trained strong classifier
  //    decrease threshold for the last classifier until the current *cascaded classifier* has a detection rate of at least d * D_i-1
  //  empty set of negative example
  //  if F_i > F_target
  //    evaluate current cascaded detector on set of non-face and put any false detection into the set of negative examples

  // f = maximum acceptable false positive rate per layer
  // d = minimum acceptable detection rate per layer
  // F_target = target overall false positive rate
  // F = 1.0 = current false positive rate
  // while F > F_target
  //  F_tmp = F
  //  n_i = 0
  //  while F_tmp > f * F (or "while FPR of the layer > f" ?)
  //    n_i++
  //    train a strong classifier with n_i features
  //    evaluate F_tmp and D_i for trained *cascaded classifier*
  //    decrease threshold for the last classifier until the current *cascaded classifier* has a detection rate of at least d * D_i-1
  //  F = F_tmp

  return 0;
}
