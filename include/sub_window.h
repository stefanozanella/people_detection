#ifndef SUB_WINDOW_H
#define SUB_WINDOW_H

#include "sample.h"
#include "integral_image.h"

class SubWindow : public Sample {
  int x_offset, y_offset, window_size;
  float subwindow_variance;
  IntegralImage integral_image;

  int scaled_coordinate(int coordinate, uint32_t base_size) const;

  public:

  SubWindow(PointCloudT::Ptr cloud);

  void crop(int from_x, int from_y, int win_size);
  float area_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const;
};

#endif
