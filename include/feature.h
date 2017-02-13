#ifndef FEATURE_H
#define FEATURE_H

#include <vector>
#include <iostream>

#include "sample.h"
#include "rect.h"
#include "storage.h"

using std::vector;

class Feature {
  public:
  vector<Rect> rectangles;

  Feature(uint32_t base_size);
  Feature& operator<<(const Rect& rectangle);
  float apply(const Sample& sample) const;
  void save(Storage& storage) const;

  friend std::ostream& operator<<(std::ostream& os, const Feature& feature);

  private:
  int base_size;
};

#endif
