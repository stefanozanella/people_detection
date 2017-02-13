#include "training_sample.h"

TrainingSample::TrainingSample(PointCloudT::Ptr cloud, bool is_positive) :
  integral_image (IntegralImage(cloud)),
  is_positive (is_positive),
  weight (1.0),
  size (cloud->width) // invariant: cloud->width == cloud->height
{
  variance = integral_image.variance(0, 0, cloud->width-1, cloud->height-1);
}

/**
 * Apply variance normalization to the given area sum.
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
 *
 * When applying variance normalization to a sum of pixels, though, we can use
 * the simplified expression:
 *
 * norm_sum = sum(X) / s^2
 */
float TrainingSample::area_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const {
  return
    integral_image.sum(
      scaled_coordinate(from_x, base_size),
      scaled_coordinate(from_y, base_size),
      scaled_coordinate(to_x, base_size),
      scaled_coordinate(to_y, base_size)
    ) / variance;
}

int TrainingSample::scaled_coordinate(int coordinate, uint32_t base_size) const {
  return round(coordinate * size / base_size);
}

