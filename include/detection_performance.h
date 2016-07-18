#ifndef DETECTION_PERFOMANCE_H
#define DETECTION_PERFOMANCE_H

#include <vector>

#include "training_sample.h"
#include "strong_classifier.h"
#include "detection_stats.h"

using std::vector;

class DetectionPerformance {
  vector<TrainingSample>& samples;
  StrongClassifier& classifier;

  public:

  DetectionPerformance(vector<TrainingSample>& samples, StrongClassifier& classifier);
  DetectionStats analyze();
};

#endif
