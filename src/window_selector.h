#ifndef _WINDOW_SELECTOR_H
#define _WINDOW_SELECTOR_H

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> RGBHandler;
typedef pcl::ExtractIndices<PointT> ExtractIndices;
typedef pcl::visualization::PointPickingEvent PointPickingEvent;
typedef pcl::search::KdTree<PointT> KdTree;

class WindowSelector {
  PointCloudT::Ptr input;
  PCLVisualizer viewer;
  KdTree search;
  PointCloudT::Ptr window;
  ExtractIndices window_extractor;
  static const int win_size = 64; // TODO Ahem...

  void relocateWindow(const PointPickingEvent &event, void*);

  public:

  WindowSelector(const PointCloudT::Ptr input);
  ~WindowSelector();
  void spin();
};

#endif
