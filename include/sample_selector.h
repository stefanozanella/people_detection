#ifndef _SAMPLE_SELECTOR_H
#define _SAMPLE_SELECTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> RGBHandler;
typedef pcl::ExtractIndices<PointT> ExtractIndices;
typedef pcl::visualization::PointPickingEvent PointPickingEvent;
typedef pcl::visualization::KeyboardEvent KeyboardEvent;
typedef pcl::search::KdTree<PointT> KdTree;

using std::string;
using std::vector;

class SampleSelector {
  static const int base_win_size = 148; // TODO Ahem...

  PointCloudT::Ptr input;
  PCLVisualizer viewer;
  KdTree search;
  PointCloudT::Ptr window;
  ExtractIndices window_extractor;
  uint32_t window_x, window_y;
  int win_size;
  vector<PointCloudT::Ptr> _faces;

  void relocateWindow(const PointPickingEvent &event, void*);
  void keyboardInteraction(const KeyboardEvent &event, void*);
  void shiftWindow(const string &direction);
  void updateWindow();
  void saveCurrentWindow();

  public:

  SampleSelector();
  ~SampleSelector();
  void setInputCloud(const PointCloudT::Ptr input);
  void spin();
  vector<PointCloudT::Ptr> faces();
};

#endif
