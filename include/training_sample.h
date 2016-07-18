#ifndef TRAINING_SAMPLE_H
#define TRAINING_SAMPLE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sample.h"
#include "integral_image.h"

class TrainingSample : public Sample {
  IntegralImage integral_image;
  float variance;

  int scaled_coordinate(int coordinate, uint32_t base_size) const;

  public:

  // TODO Camel case?
  bool isPositive;
  float weight;
  uint32_t size;

  TrainingSample(PointCloudT::Ptr cloud, bool isPositive);
  float area_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const;
};

#endif
