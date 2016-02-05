#include "training_sample.h"

#include <pcl/point_types_conversion.h>

TrainingSample::TrainingSample(PointCloudT::Ptr cloud, bool isPositive) :
  cloud (cloud),
  isPositive (isPositive),
  integral_image (new MonochromePointCloudT)
{
  calculateIntegralImage();
}

TrainingSample::~TrainingSample() {}

/**
 * Calculates integral image. At a specific location (j,k), the integral image
 * can be computed as the algebraic addition the follows:
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
void TrainingSample::calculateIntegralImage() {
  integral_image->width = cloud->width;
  integral_image->height = cloud->height;
  integral_image->points.resize(cloud->points.size());
  integral_image->is_dense = cloud->is_dense;

  for (int k = 0; k < cloud->height; k++) {
    for (int j = 0; j < cloud->width; j++) {
      // TODO What if 0,0 is NaN? What if another point is Nan?
      // TODO A: I don't care because all that matters are intensities for now.
      // TODO Hint: colors might be NaN too.
      MonochromePointT d;
      pcl::PointXYZRGBtoXYZI(cloud->at(j,k), d);

      float a = j < 1 || k < 1 ? 0 : integral_image->at(j-1, k-1).intensity;
      float b = j < 1 ? 0 : integral_image->at(j-1, k).intensity;
      float c = k < 1 ? 0 : integral_image->at(j, k-1).intensity;

      d.intensity += b + c - a;
      integral_image->at(j, k) = d;
    }
  }
}
