#ifndef DETECTION_STATS_H
#define DETECTION_STATS_H

struct DetectionStats {
  DetectionStats() {
    total_positive = total_negative = false_positives = false_negatives = 0;
    detection_rate = false_positive_rate = 0.0;
  }

  int total_positive, total_negative, false_positives, false_negatives;
  float detection_rate, false_positive_rate;
};

#endif
