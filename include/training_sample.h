#ifndef TRAINING_SAMPLE_H
#define TRAINING_SAMPLE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZI MonochromePointT;
typedef pcl::PointCloud<MonochromePointT> MonochromePointCloudT;

class TrainingSample {
  public:

  PointCloudT::Ptr cloud;
  MonochromePointCloudT::Ptr integral_image;
  bool isPositive;

  TrainingSample(PointCloudT::Ptr cloud, bool isPositive);
  ~TrainingSample();

  float scaled_integral_sum(int from_x, int from_y, int to_x, int to_y, uint32_t base_size) const;
  int scaled_coordinate(int coordinate, uint32_t base_size) const;

  private:

  float integral_sum(int from_x, int from_y, int to_x, int to_y) const;
  void calculateIntegralImage();
};

#endif
