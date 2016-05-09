#include "integral_image.h"

#include <pcl/point_types_conversion.h>
#include <pcl/common/io.h>
#include <cmath>

IntegralImage::IntegralImage(PointCloudT::Ptr cloud) :
  cloud (cloud),
  integral_image (new MonochromePointCloudT),
  square_integral_image (new MonochromePointCloudT)
{
  compute_integral_image(cloud, integral_image);
  compute_integral_image(cloud, square_integral_image, 2);
}

/**
 * Calculates integral image. At a specific location (j,k), the integral image
 * can be computed as the algebraic addition that follows:
 * +-------+---+
 * |       |   |
 * |   A   | B |
 * |       |   |
 * +-------+---+
 * |   C   | D |
 * +-------+---+(j,k)
 *
 * ii(j,k) = D + B + C - A
 *
 * Where:
 *
 * D = cloud[j,k]
 * B = ii(j-1, k)
 * C = ii(j, k-1)
 * A = ii(j-1, k-1)
 *
 * On the 1st row and 1st column, we must subsititute values where one or both
 * indexes are out of range with the value 0 (i.e. the particular element is
 * not to be considered). The same effect can be obtained by augmenting the
 * image with a row and a column of 0s in the first position:
 *
 * 0|00000000000|
 * -+-------+---+
 * 0|       |   |
 * 0|   A   | B |
 * 0|       |   |
 * -+-------+---+
 * 0|   C   | D |
 * -+-------+---+(j,k)
 */
void IntegralImage::compute_integral_image(PointCloudT::Ptr cloud, MonochromePointCloudT::Ptr integral_image, int power) {
  integral_image->width = cloud->width;
  integral_image->height = cloud->height;
  integral_image->points.resize(cloud->points.size());
  integral_image->is_dense = cloud->is_dense;

  for (int k = 0; k < cloud->height; k++) {
    for (int j = 0; j < cloud->width; j++) {
      // TODO What if z at 0,0 is NaN? What if another point is Nan?
      // TODO A: I don't care because all that matters are intensities for now.
      // TODO Hint: colors might be NaN too.
      MonochromePointT d;
      pcl::PointXYZRGBtoXYZI(cloud->at(j,k), d);

      float a = j && k ? integral_image->at(j-1, k-1).intensity : 0;
      float b = j ? integral_image->at(j-1, k).intensity : 0;
      float c = k ? integral_image->at(j, k-1).intensity : 0;

      d.intensity = pow(d.intensity, power) + b + c - a;
      integral_image->at(j, k) = d;
    }
  }
}

inline float IntegralImage::area_sum(MonochromePointCloudT::Ptr cloud, int from_x, int from_y, int to_x, int to_y) const {
  return
    cloud->at(to_x, to_y).intensity -
    (from_x ? cloud->at(from_x-1, to_y).intensity : 0) -
    (from_y ? cloud->at(to_x, from_y-1).intensity : 0) +
    (from_x && from_y ? cloud->at(from_x-1, from_y-1).intensity : 0);
}

float IntegralImage::sum(int from_x, int from_y, int to_x, int to_y) const {
  return area_sum(integral_image, from_x, from_y, to_x, to_y);
}

float IntegralImage::square_sum(int from_x, int from_y, int to_x, int to_y) const {
  return area_sum(square_integral_image, from_x, from_y, to_x, to_y);
}

float IntegralImage::mean(int from_x, int from_y, int to_x, int to_y) const {
  return 1.0f/((to_x - from_x) * (to_y - from_y)) * sum(from_x, from_y, to_x, to_y);
}

float IntegralImage::variance(int from_x, int from_y, int to_x, int to_y) const {
  return pow(mean(from_x, from_y, to_x, to_y), 2) -
    1.0f/((to_x - from_x) * (to_y - from_y)) *
    square_sum(from_x, from_y, to_x, to_y);
}
