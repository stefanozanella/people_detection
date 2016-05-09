#ifndef INTEGRAL_IMAGE_H
#define INTEGRAL_IMAGE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZI MonochromePointT;
typedef pcl::PointCloud<MonochromePointT> MonochromePointCloudT;

class IntegralImage {
  PointCloudT::Ptr cloud;
  MonochromePointCloudT::Ptr integral_image;
  MonochromePointCloudT::Ptr square_integral_image;

  void compute_integral_image(PointCloudT::Ptr cloud, MonochromePointCloudT::Ptr integral, int power = 1);
  float square_sum(int from_x, int from_y, int to_x, int to_y) const;
  inline float area_sum(MonochromePointCloudT::Ptr cloud, int from_x, int from_y, int to_x, int to_y) const;

  public:

  IntegralImage(PointCloudT::Ptr cloud);

  float sum(int from_x, int from_y, int to_x, int to_y) const;
  float mean(int from_x, int from_y, int to_x, int to_y) const;
  float variance(int from_x, int from_y, int to_x, int to_y) const;
};

#endif
