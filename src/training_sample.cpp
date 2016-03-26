#include "training_sample.h"

#include <pcl/point_types_conversion.h>
#include <cmath>

TrainingSample::TrainingSample(PointCloudT::Ptr cloud, bool isPositive) :
  cloud (cloud),
  isPositive (isPositive),
  integral_image (new MonochromePointCloudT),
  weight (1.0)
{
  calculateIntegralImage(cloud, integral_image);
  normalize(integral_image);
}

TrainingSample::~TrainingSample() {}

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
void TrainingSample::calculateIntegralImage(PointCloudT::Ptr cloud, MonochromePointCloudT::Ptr integral_image) {
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

      float a = j < 1 || k < 1 ? 0 : integral_image->at(j-1, k-1).intensity;
      float b = j < 1 ? 0 : integral_image->at(j-1, k).intensity;
      float c = k < 1 ? 0 : integral_image->at(j, k-1).intensity;

      d.intensity += b + c - a;
      integral_image->at(j, k) = d;
    }
  }
}

/**
 * Apply variance normalization to the given image.
 *
 * Variance normalization transforms the input image so that the result has a
 * N(0,1) distribution.
 *
 * Given image's mean value:
 *
 * m = 1/N * sum(X)
 *
 * and square variance:
 *
 * s^2 = m^2 - 1/N * sum(X^2)
 *
 * then the value of a pixel x can be normalized with:
 *
 * x_norm = (x - m)/s^2
 */
void TrainingSample::normalize(MonochromePointCloudT::Ptr image) {
  float mean = 1.0f/(image->width * image-> height) * image->at(image->width-1, image->height-1).intensity;

  float square_sum = 0;
  for (int k = 0; k < image->height; k++) {
    for (int j = 0; j < image->width; j++) {
      square_sum += pow(image->at(j,k).intensity, 2);
    }
  }

  float variance = pow(mean, 2) - 1.0f/(image->width * image->height) * square_sum;

  for (int k = 0; k < image->height; k++) {
    for (int j = 0; j < image->width; j++) {
      image->at(j, k).intensity = (image->at(j, k).intensity - mean) / variance;
    }
  }
}

float TrainingSample::scaled_integral_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const {
  return
    integral_sum(
      scaled_coordinate(from_x, base_size),
      scaled_coordinate(from_y, base_size),
      scaled_coordinate(to_x, base_size),
      scaled_coordinate(to_y, base_size)
    );
}

float TrainingSample::integral_sum(int from_x, int from_y, int to_x, int to_y) const {
  return
    integral_image->at(to_x, to_y).intensity -
    integral_image->at(from_x, to_y).intensity -
    integral_image->at(to_x, from_y).intensity +
    integral_image->at(from_x, from_y).intensity;
}

int TrainingSample::scaled_coordinate(int coordinate, uint32_t base_size) const {
  return round(coordinate * integral_image->width / base_size);
}
