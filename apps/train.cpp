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
#include "strong_classifier_training.h"
#include "detection_performance.h"
#include "cascade_classifier.h"

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
  const int base_win_size = 148;

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

void save_training_results(CascadeClassifier& classifier, const string& file) {
  Storage storage;
  classifier.save(storage);
  storage.persist(file);
}

void show_performance_stats(DetectionStats stats) {
  cout << "Current detection stats" << endl;
  cout << "\tTotal positive samples: " << stats.total_positive << endl;
  cout << "\tTotal negative samples: " << stats.total_negative << endl;
  cout << "\tDetected false negatives: " << stats.false_negatives << endl;
  cout << "\tDetected false positives: " << stats.false_positives << endl;
  cout << "\tDetection rate: " << stats.detection_rate << endl;
  cout << "\tFalse positive rate: " << stats.false_positive_rate << endl;
}

int main(int argc, char** argv) {
  const float MINIMUM_DETECTION_RATE = 0.85;
  const float MAXIMUM_FALSE_POSITIVE_RATE = 0.45;
  const float TARGET_FALSE_POSITIVE_RATE = 0.01;
  const float ADJUSTMENT_RATIO = 0.1;
  float current_false_positive_rate = 1.0, current_detection_rate = 0.0;

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

  vector<TrainingSample> positive_training_samples,
    negative_training_sample_pool,
    samples,
    validation_set;

  for (int k = 0; k < positive_samples.size(); k++) {
    positive_training_samples.push_back(TrainingSample(positive_samples.at(k), true));
    validation_set.push_back(TrainingSample(positive_samples.at(k), true));
  }

  for (int k = 0; k < negative_sample_pool.size(); k++) {
    negative_training_sample_pool.push_back(TrainingSample(negative_sample_pool.at(k), false));
    validation_set.push_back(TrainingSample(negative_sample_pool.at(k), false));
  }

  int step = max((int)(negative_sample_pool.size() / positive_samples.size()), 1);
  for (int k = 0; k < positive_samples.size(); k++) {
    samples.push_back(positive_training_samples.at(k));
    samples.push_back(negative_training_sample_pool.at(k*step % negative_sample_pool.size()));
  }

  cout << "Generating features...";
  generate_features(features, find_smallest_window(samples));
  cout << features.size() << " features generated." << endl;

  CascadeClassifier cascade;

  float last_stage_false_positive_rate = current_false_positive_rate;
  float last_stage_detection_rate = current_detection_rate;

  while (current_false_positive_rate > TARGET_FALSE_POSITIVE_RATE) {
    cout << "Training new stage" << endl;

    float last_false_positive_rate = current_false_positive_rate;
    float last_detection_rate = current_detection_rate;

    StrongClassifier strong;
    StrongClassifierTraining training (samples, features, strong);

    cascade.push_back(strong);

    // TODO
    // We should check also here if we're improving the DR with respect to the
    // previous iteration. If not, we should get out of the loop
    while (current_false_positive_rate > MAXIMUM_FALSE_POSITIVE_RATE * last_false_positive_rate) {
      float last_confirmed_detection_rate = current_detection_rate;
      float last_confirmed_false_positive_rate = current_false_positive_rate;

      cout << "\tTraining new weak classifier" << endl;
      cascade.pop_back();

      training.trainWeakClassifier(MINIMUM_DETECTION_RATE * last_detection_rate);

      cascade.push_back(strong);

      DetectionPerformance validation_performance (validation_set, cascade);
      DetectionStats validation_stats = validation_performance.analyze();
      show_performance_stats(validation_stats);

      cout << "Last FPR: " << current_false_positive_rate << endl;
      if (validation_stats.false_positive_rate >= current_false_positive_rate) {
        cascade.pop_back();
        training.discard_last_trained_classifier();
        cascade.push_back(strong);

        cout << "Breaking the loop" << endl;
        break;
      }

      float reference_false_positive_rate = current_false_positive_rate;

      current_false_positive_rate = validation_stats.false_positive_rate;
      current_detection_rate = validation_stats.detection_rate;

      float reference_detection_rate = current_detection_rate;

      while (current_detection_rate < MINIMUM_DETECTION_RATE) {
        cout << "Current detection rate: " << current_detection_rate << " | Wanted detection rate: " << MINIMUM_DETECTION_RATE * last_detection_rate << endl;

        reference_false_positive_rate = current_false_positive_rate;

        cascade.pop_back();
        training.adjust_threshold(ADJUSTMENT_RATIO);
        cascade.push_back(strong);

        DetectionPerformance adjustment_performance (validation_set, cascade);
        DetectionStats adjustment_stats = adjustment_performance.analyze();
        show_performance_stats(adjustment_stats);

        bool is_improving = adjustment_stats.detection_rate > current_false_positive_rate;

        current_false_positive_rate = adjustment_stats.false_positive_rate;
        current_detection_rate = adjustment_stats.detection_rate;

        if (!is_improving) {
          cascade.pop_back();

          cout << "Adjustment is not improving performance" << endl;
          break;
        }
      }

      if (current_detection_rate <= reference_detection_rate && current_false_positive_rate >= reference_false_positive_rate) {
        cascade.pop_back();
        training.discard_last_trained_classifier();
        cascade.push_back(strong);

        break;
      }

      DetectionPerformance final_validation_performance (validation_set, cascade);
      DetectionStats final_validation_stats = final_validation_performance.analyze();

      cout << endl;

      if (current_detection_rate <= last_confirmed_detection_rate && current_false_positive_rate >= last_confirmed_false_positive_rate) {
        cascade.pop_back();
        training.discard_last_trained_classifier();
        cascade.push_back(strong);

        break;
      }


      current_false_positive_rate = final_validation_stats.false_positive_rate;
      current_detection_rate = final_validation_stats.detection_rate;
    }


    // TODO We need to store the last performances at the end of each weak
    // classifier training
    cout << "\t\tCurrent DR: " << current_detection_rate << " | Last stage DR: " << last_stage_detection_rate << endl;
    cout << "\t\tCurrent FPR: " << current_false_positive_rate << " | Last FPR: " << last_stage_false_positive_rate << endl;

    if (current_false_positive_rate >= last_stage_false_positive_rate && current_detection_rate <= last_stage_detection_rate) {
      cout << "Cannot improve performances further. Giving up." << endl;
      cascade.pop_back();
      break;
    }

    last_stage_false_positive_rate = current_false_positive_rate;
    last_stage_detection_rate = current_detection_rate;

    samples.clear();
    for (int k = 0; k < positive_training_samples.size(); k++) {
      samples.push_back(positive_training_samples.at(k));
    }

    int j = 0, k = 0;
    while (k < positive_training_samples.size()) {
      if (j >= negative_training_sample_pool.size()) {
        break;
      }

      if (cascade.is_face(negative_training_sample_pool.at(j))) {
        samples.push_back(negative_training_sample_pool.at(j));
        k++;
      }
      j++;
    }
  }

  save_training_results(cascade, "face_detector.yml");

  return 0;
}
