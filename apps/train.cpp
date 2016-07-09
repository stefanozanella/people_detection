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
 * [ ] randomly pick out N from the non-face subwindows
 * [ ] train first classifier
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

using std::string;
using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using pcl::io::loadPCDFile;
using std::max;

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

  // TODO the trainer should wrap the samples so that it "owns" the weights.
  StrongClassifierTraining training (samples, features, strong);

  training.trainWeakClassifier();
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
  //    train a strong classifier with n_i features (-> add a trained weak
  //    classifier to the previously trained strong classifier)
  //    evaluate F_tmp and D_i for trained *cascaded classifier*
  //    decrease threshold for the last classifier until the current *cascaded classifier* has a detection rate of at least d * D_i-1
  //  F = F_tmp

  return 0;
}
