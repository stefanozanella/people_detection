#include "cascade_classifier.h"

CascadeClassifier::CascadeClassifier() {}

CascadeClassifier& CascadeClassifier::operator<<(const StrongClassifier& strong) {
  cascade.push_back(strong);
}

bool CascadeClassifier::is_face(const Sample& sample) const {
  bool is_face = true;
  int k = 0;

  while (is_face && k < cascade.size()) {
    is_face = cascade.at(k).is_face(sample);
    k++;
  }

  return is_face;
}

void CascadeClassifier::save(Storage& storage) const {
  for (vector<StrongClassifier>::const_iterator stage = cascade.begin(); stage != cascade.end(); stage++) {
    Storage stages;
    stage->save(stages);
    storage.push_back("stages", stages);
  }
}
