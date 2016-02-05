#ifndef _TRAINING_SAMPLE_H
#define _TRAINING_SAMPLE_H

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

  private:

  void calculateIntegralImage();
};

#endif
