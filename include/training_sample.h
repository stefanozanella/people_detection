#ifndef _TRAINING_SAMPLE_H
#define _TRAINING_SAMPLE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class TrainingSample {
  public:

  PointCloudT::Ptr cloud;
  bool isPositive;

  TrainingSample(bool isPositive);
  ~TrainingSample();
};

#endif
