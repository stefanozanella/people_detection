#ifndef CASCADE_CLASSIFIER_H
#define CASCADE_CLASSIFIER_H

#include <vector>

#include "strong_classifier.h"
#include "sample.h"

using std::vector;

class CascadeClassifier {
  vector<StrongClassifier> cascade;

  public:
  CascadeClassifier();
  CascadeClassifier& operator<<(const StrongClassifier& strong);
  bool is_face(const Sample& sample) const;
  void save(Storage& storage) const;
};

#endif

