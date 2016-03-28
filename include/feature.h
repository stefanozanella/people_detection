#ifndef FEATURE_H
#define FEATURE_H

#include <vector>
#include <iostream>

#include "training_sample.h"
#include "rect.h"

using std::vector;

class Feature {
  public:
  vector<Rect> rectangles; // TODO This might become private again

  Feature(uint32_t base_size);
  Feature& operator<<(const Rect& rectangle);
  float apply(const TrainingSample& sample) const;

  friend std::ostream& operator<<(std::ostream& os, const Feature& feature);

  private:
  int base_size;
};

#endif
