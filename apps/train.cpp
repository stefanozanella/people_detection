/**
 * GOAL:
 * [x] load samples
 * [x] calculate samples's integral image
 * [x] samples normalization
 * [x] generate features
 * [x] verify generated features and feature calculation
 * [x] select first weak classifier
 *   [x] for each feature
 *     [x] compute feature value for all samples
 *     [x] sort feature values
 *     [x] compute optimal threshold and polarity
 *     TODO Refactor and document
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

using std::string;
using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using pcl::io::loadPCDFile;
using std::max;
using std::sort;

int main(int argc, char** argv) {
  string positive_samples_dir (argv[1]), negative_samples_dir (argv[2]);

  directory_iterator positive_sample ((path(positive_samples_dir)));
  directory_iterator negative_sample ((path(negative_samples_dir)));
  directory_iterator no_more_samples;

  vector<TrainingSample> samples;

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

  uint32_t base_win_size = 0;
  // Invariants:
  // - samples are square in size (width == height)
  // - samples are organized
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    base_win_size = max(base_win_size, sample->cloud->width);
  }

  cout << "Base win size for the training set: " << base_win_size << endl;

  int count = 0;

  vector<Feature> features;

  for (uint32_t x = 0; x < base_win_size; x++) {
    for (uint32_t y = 0; y < base_win_size; y++) {
      // generate horizontal haar2
      for (uint32_t dx = 1; 2*dx < base_win_size - x; dx++) {
        for (uint32_t dy = 1; dy < base_win_size - y; dy++) {
          features.push_back(
            Feature(base_win_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1)
          );
          count++;
        }
      }

      // generate vertical haar2
      for (int dx = 1; dx < base_win_size - x; dx++) {
        for (int dy = 1; 2*dy < base_win_size - y; dy++) {
          features.push_back(
            Feature(base_win_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x, y + dy, dx, dy, 1)
          );
          count++;
        }
      }

      // generate horizontal haar3
      for (int dx = 1; 3*dx < base_win_size - x; dx++) {
        for (int dy = 1; dy < base_win_size - y; dy++) {
          features.push_back(
            Feature(base_win_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1) <<
            Rect(x + 2*dx, y, dx, dy, -1)
          );
          count++;
        }
      }

      // generate vertical haar3
      for (int dx = 1; dx < base_win_size - x; dx++) {
        for (int dy = 1; 3*dy < base_win_size - y; dy++) {
          features.push_back(
            Feature(base_win_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x, y + dy, dx, dy, 1) <<
            Rect(x, y + 2*dy, dx, dy, -1)
          );
          count++;
        }
      }

      // generate haar4
      for (int dx = 1; 4*dx < base_win_size - x; dx++) {
        for (int dy = 1; 4*dy < base_win_size - y; dy++) {
          features.push_back(
            Feature(base_win_size) <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1) <<
            Rect(x, y + dy, dx, dy, 1) <<
            Rect(x + dx, y + dy, dx, dy, -1)
          );
          count++;
        }
      }
    }
  }

  cout << "Generated " << count << " features" << endl;

  Feature* optimal_feature;
  float optimal_threshold, optimal_error = FLT_MAX;
  int optimal_polarity;

  for (int k = 0; k < features.size(); k += 1) {
    //Feature feature = features.at(16060800);
    Feature feature = features.at(k);

    vector<FeatureValue> feature_values;

    for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
      FeatureValue feature_value(feature, *sample);
      feature_values.push_back(feature_value);
    }

    sort(feature_values.begin(), feature_values.end());

    float total_positive_weight_sum = 0, total_negative_weigth_sum = 0, positive_below = 0, negative_below = 0;

    // TODO Use actual weights. Compute these two while updating the weights
    for (vector<FeatureValue>::iterator feature_value = feature_values.begin(); feature_value != feature_values.end(); feature_value++) {
      total_positive_weight_sum += feature_value->sample.isPositive;
      total_negative_weigth_sum += 1 - feature_value->sample.isPositive;
    }

    //cout << "Current T+ and T-: " << total_positive_weight_sum << " - " << total_negative_weigth_sum << endl;

    float optimal_threshold_for_this_feature, optimal_error_for_this_feature = FLT_MAX;
    int optimal_polarity_for_this_feature;
    float previous_feature_value = 0;

    for (vector<FeatureValue>::iterator feature_value = feature_values.begin(); feature_value != feature_values.end(); feature_value++) {
      //cout << "********************************" << endl;

      //cout << "Feature value: " << feature_value->value << endl;
      //cout << "S+: " << positive_below << " - S-: " << negative_below << endl;
      //cout << "Is positive? " << feature_value->sample.isPositive << endl;

      float error_when_polarity_is_negative = positive_below + total_negative_weigth_sum - negative_below;
      float error_when_polarity_is_positive = negative_below + total_positive_weight_sum - positive_below;

      //cout << "Error p > 0: " << error_when_polarity_is_positive << endl;
      //cout << "Error p < 0: " << error_when_polarity_is_negative << endl;

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

      if (optimal_error_for_this_sample < optimal_error_for_this_feature) {
        optimal_error_for_this_feature = optimal_error_for_this_sample;
        optimal_polarity_for_this_feature = optimal_polarity_for_this_sample;
        optimal_threshold_for_this_feature = optimal_threshold_for_this_sample;
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
      positive_below += feature_value->sample.isPositive && (feature_value->value > previous_feature_value);
      negative_below += (1 - feature_value->sample.isPositive) && (feature_value->value > previous_feature_value);
      previous_feature_value = feature_value->value;

      //cout << "********************************" << endl;
    }

    //cout << "Optimal threshold: " << optimal_threshold_for_this_feature << endl;
    //cout << "Optimal polarity: " << optimal_polarity_for_this_feature << endl;
    //cout << "Optimal error: " << optimal_error_for_this_feature << endl;
    //cout << "################################" << endl;

    if (optimal_error_for_this_feature < optimal_error) {
      optimal_feature = &feature;
      optimal_error = optimal_error_for_this_feature;
      optimal_threshold = optimal_threshold_for_this_feature;
      optimal_polarity = optimal_polarity_for_this_feature;
    }
  }

  cout << "Final threshold: " << optimal_threshold << endl;
  cout << "Final polarity: " << optimal_polarity << endl;
  cout << "Final error: " << optimal_error << endl;

  return 0;
}
