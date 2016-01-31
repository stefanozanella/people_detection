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
typedef pcl::visualization::KeyboardEvent KeyboardEvent;
typedef pcl::search::KdTree<PointT> KdTree;

class WindowSelector {
  PointCloudT::Ptr input;
  PCLVisualizer viewer;
  KdTree search;
  PointCloudT::Ptr window;
  ExtractIndices window_extractor;
  static const int win_size = 64; // TODO Ahem...
  uint32_t window_x, window_y;

  void relocateWindow(const PointPickingEvent &event, void*);
  void shiftWindow(const KeyboardEvent &event, void*);
  void updateWindow();

  public:

  WindowSelector(const PointCloudT::Ptr input);
  ~WindowSelector();
  void spin();
};

#endif
