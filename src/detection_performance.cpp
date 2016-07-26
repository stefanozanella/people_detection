#include "detection_performance.h"

DetectionPerformance::DetectionPerformance(vector<TrainingSample>& samples, CascadeClassifier& classifier) :
  samples (samples),
  classifier (classifier)
{}

DetectionStats DetectionPerformance::analyze() {
  DetectionStats stats;

  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    if (sample->isPositive) {
      stats.total_positive++;

      if (!classifier.is_face(*sample))
        stats.false_negatives++;
    }
    else {
      stats.total_negative++;

      if (classifier.is_face(*sample))
        stats.false_positives++;
    }
  }

  stats.false_positive_rate = 1.0f * stats.false_positives / stats.total_negative;
  stats.detection_rate = 1 - (1.0f * stats.false_negatives) / stats.total_positive;

  return stats;
}
