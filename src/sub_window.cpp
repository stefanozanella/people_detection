#include "sub_window.h"

SubWindow::SubWindow(PointCloudT::Ptr cloud) :
  integral_image (IntegralImage(cloud)),
  x_offset (0),
  y_offset (0),
  window_size (cloud->width) // invariant: cloud->width == cloud->height
{
  subwindow_variance = integral_image.variance(0, 0, cloud->width-1, cloud->height-1);
}

void SubWindow::crop(int from_x, int from_y, int win_size) {
  x_offset = from_x;
  y_offset = from_y;
  window_size = win_size;
  subwindow_variance = integral_image.variance(from_x, from_y, from_x + win_size - 1, from_y + win_size - 1);
}

float SubWindow::area_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const {
  return
    integral_image.sum(
      scaled_coordinate(from_x, base_size) + x_offset,
      scaled_coordinate(from_y, base_size) + y_offset,
      scaled_coordinate(to_x, base_size) + x_offset,
      scaled_coordinate(to_y, base_size) + y_offset
    ) / subwindow_variance;
}


int SubWindow::scaled_coordinate(int coordinate, uint32_t base_size) const {
  return round(coordinate * window_size / base_size);
}
