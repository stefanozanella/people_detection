#ifndef DETECTION_PERFOMANCE_H
#define DETECTION_PERFOMANCE_H

#include <vector>

#include "training_sample.h"
#include "cascade_classifier.h"
#include "detection_stats.h"

using std::vector;

class DetectionPerformance {
  vector<TrainingSample>& samples;
  CascadeClassifier& classifier;

  public:

  DetectionPerformance(vector<TrainingSample>& samples, CascadeClassifier& classifier);
  DetectionStats analyze();
};

#endif
