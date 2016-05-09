#ifndef SAMPLE_H
#define SAMPLE_H

#include <pcl/point_types.h>

class Sample {
  public:

  virtual float area_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const = 0;
};

#endif
