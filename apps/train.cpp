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
 *   [x] add more positive samples
 *   [x] add more negative samples
 * [ ] decide on a base size for the training set, split 50:50 (probably
 * determined by how many faces you can pick out)
 * [x] scan image without faces -> use algo from people_detector
 * [ ] find base size, pick max together with positive samples as a base size
 *     for feature generation
 * [x] pick randomly from negative samples pool, fill up to match number of
 *     faces -> not really random, but doesn't change results much apparently
 * [ ] train first classifier
 *   [x] fix polarity calculation in StrongClassifierTraining
 *   [x] fix feature generation, base it on the minimum example's size (you
 *   shouldn't have any feature value = 0)
 *   [ ] fix forced detection algorithm
 * [ ] prepare data for second round
 *   [ ] reset positive samples so that weight are reinitialized
 *   [ ] pick randomly from validation samples that fail classification
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
#include "weak_classifier.h"
#include "strong_classifier.h"
#include "storage.h"
#include "load_trained_detector.h"
#include "strong_classifier_training.h"
#include "detection_performance.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using pcl::io::loadPCDFile;
using std::max;
using std::min;

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

int load_positive_samples(vector<PointCloudT::Ptr>& samples, const string& source) {
  directory_iterator positive_sample ((path(source)));
  directory_iterator no_more_samples;

  for (; positive_sample != no_more_samples; positive_sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(positive_sample->path().string(), *(sample_cloud)) == -1) {
      return -1;
    }

    samples.push_back(sample_cloud);
  }

  return 0;
}

int scan_samples_from_point_cloud(vector<PointCloudT::Ptr>& samples, const string& source, const int step) {
  PointCloudT::Ptr cloud (new PointCloudT);

  if (loadPCDFile<PointT>(source, *cloud) == -1) {
    return -1;
  }

  int x = 0, y = 0, current_win_size;
  const int base_win_size = 148; // TODO

  int pos = 0, neg = 0;

  while (y < cloud->height) {
    while (x < cloud->width) {
      float center = cloud->at(x, y).z;

      if (pcl_isnan(center)) {
        x++;
        continue;
      }

      current_win_size = ceil(base_win_size / cloud->at(x, y).z);

      int x_from = x - floor(current_win_size / 2.0);
      int x_to = x + ceil(current_win_size / 2.0);
      int y_from = y - floor(current_win_size / 2.0);
      int y_to = y + ceil(current_win_size / 2.0);

      if (x_from >= 0 && x_to < cloud->width && y_from >= 0 && y_to < cloud->height) {
        vector<int> indices;
        for (int j = y_from; j < y_to; j++) {
          for (int k = x_from; k < x_to; k++) {
            indices.push_back(j*cloud->width + k);
          }
        }
        PointCloudT::Ptr sample (new PointCloudT);
        copyPointCloud(*cloud, indices, *sample);
        sample->width = sample->height = current_win_size;
        samples.push_back(sample);
      }
      x+=step;
    }
    y+=step;
    x = 0;
  }

  return 0;
}

/**
 * Invariants:
 * * samples are square in size (width == height)
 * * samples are organized clouds
 */
uint32_t find_smallest_window(const vector<TrainingSample>& samples) {
  uint32_t smallest_window_size = INT_MAX;
  for (vector<TrainingSample>::const_iterator sample = samples.begin(); sample != samples.end(); sample++) {
    smallest_window_size = min(smallest_window_size, sample->size);
  }

  return smallest_window_size;
}

void save_training_results(StrongClassifier& classifier, const string& file) {
  Storage storage;
  classifier.save(storage);
  storage.persist(file);
}

void show_performance_stats(vector<TrainingSample>& samples, StrongClassifier& classifier) {
  DetectionPerformance performance (samples, classifier);
  DetectionStats stats = performance.analyze();

  cout << "Current detection stats" << endl;
  cout << "\tTotal positive samples: " << stats.total_positive << endl;
  cout << "\tTotal negative samples: " << stats.total_negative << endl;
  cout << "\tDetected false negatives: " << stats.false_negatives << endl;
  cout << "\tDetected false positives: " << stats.false_positives << endl;
  cout << "\tDetection rate: " << stats.detection_rate << endl;
  cout << "\tFalse positive rate: " << stats.false_positive_rate << endl;
}

int main(int argc, char** argv) {
  vector<Feature> features;

  vector<PointCloudT::Ptr> positive_samples, negative_sample_pool;

  cout << "Loading positive samples...";
  if (load_positive_samples(positive_samples, string(argv[1])) < 0) {
    cout << "Couldn't load positive samples from " << string(argv[1]) << endl;
    return -1;
  }
  cout << positive_samples.size() << " loaded." << endl;

  cout << "Scanning for negative samples...";
  if (scan_samples_from_point_cloud(negative_sample_pool, string(argv[2]), 10) < 0) {
    cout << "Couldn't load negative samples from " << string(argv[2]) << endl;
    return -1;
  }
  cout << negative_sample_pool.size() << " found." << endl;

  vector<TrainingSample> positive_training_samples, samples;

  for (int k = 0; k < positive_samples.size(); k++) {
    positive_training_samples.push_back(TrainingSample(positive_samples.at(k), true));
  }

  int step = max((int)(negative_sample_pool.size() / positive_samples.size()), 1);
  for (int k = 0; k < positive_samples.size(); k++) {
    samples.push_back(positive_training_samples.at(k));
    samples.push_back(TrainingSample(negative_sample_pool.at(k*step % negative_sample_pool.size()), false));
  }

  // TODO Move somewhere else, use point clouds rather than training samples so
  // we can do this at the very beginning
  cout << "Generating features...";
  generate_features(features, find_smallest_window(samples));
  cout << features.size() << " features generated." << endl;

  StrongClassifier strong;

  // TODO the trainer should wrap the samples so that it "owns" the weights.
  StrongClassifierTraining training (samples, features, strong);
  training.trainWeakClassifier();

  show_performance_stats(samples, strong);
  vector<TrainingSample> validation_set;
  for (int k = 0; k < negative_sample_pool.size(); k++) {
    validation_set.push_back(TrainingSample(negative_sample_pool.at(k), false));
  }
  show_performance_stats(validation_set, strong);

  // perform detection with current strong classifier on set composed of:
  // - all positive samples
  // - false positives from previous stage
  // show stats for:
  // - detection rate: ???
  // - false positive rate: ???
  // show prompt to decide if:
  // - add another weak classifier
  // - decrease threshold -> find a way to reset threshold to previous value
  // in case stats go awry?
  // - start training of new stage

  // TODO This thing needs a lot of refinement
  //for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
  //  if (sample->isPositive && !strong.is_face(*sample)) {
  //    strong.force_detection(*sample);

  //    stats = performance.analyze();

  //    cout << "Current detection stats" << endl;
  //    cout << "\tTotal positive samples: " << stats.total_positive << endl;
  //    cout << "\tTotal negative samples: " << stats.total_negative << endl;
  //    cout << "\tDetected false negatives: " << stats.false_negatives << endl;
  //    cout << "\tDetected false positives: " << stats.false_positives << endl;
  //    cout << "\tDetection rate: " << stats.detection_rate << endl;
  //    cout << "\tFalse positive rate: " << stats.false_positive_rate << endl;
  //  }
  //}

  //stats = performance.analyze();

  //cout << "Current detection stats" << endl;
  //cout << "\tTotal positive samples: " << stats.total_positive << endl;
  //cout << "\tTotal negative samples: " << stats.total_negative << endl;
  //cout << "\tDetected false negatives: " << stats.false_negatives << endl;
  //cout << "\tDetected false positives: " << stats.false_positives << endl;
  //cout << "\tDetection rate: " << stats.detection_rate << endl;
  //cout << "\tFalse positive rate: " << stats.false_positive_rate << endl;

  save_training_results(strong, "face_detector.yml");

  //cout << "~~~~~~~~~~~" << endl;

  //for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
  //  if (sample->isPositive && !strong.is_face(*sample)) {
  //    cout << "Ouch! False negative" << endl;
  //  }

  //  if (!sample->isPositive && strong.is_face(*sample)) {
  //    cout << "Meh! False positive" << endl;
  //  }
  //}

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
  //    train a strong classifier with n_i features (-> add a trained weak
  //    classifier to the previously trained strong classifier)
  //    evaluate F_tmp and D_i for trained *cascaded classifier*
  //    decrease threshold for the last classifier until the current *cascaded classifier* has a detection rate of at least d * D_i-1
  //  F = F_tmp

  return 0;
}
