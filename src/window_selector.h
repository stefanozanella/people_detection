#ifndef _WINDOW_SELECTOR_H
#define _WINDOW_SELECTOR_H

#include <pcl/point_types.h>
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

class PointCloudWindow {
  public:

  uint32_t x, y, size;
  PointCloudWindow(uint32_t x, uint32_t y, uint32_t size);
};

class WindowSelector {
  PointCloudT::Ptr input;
  PCLVisualizer viewer;
  KdTree search;
  PointCloudT::Ptr window;
  ExtractIndices window_extractor;
  static const int win_size = 64; // TODO Ahem...
  uint32_t window_x, window_y;
  vector<PointCloudWindow> _faces;

  void relocateWindow(const PointPickingEvent &event, void*);
  void keyboardInteraction(const KeyboardEvent &event, void*);
  void shiftWindow(const string &direction);
  void updateWindow();
  void saveCurrentWindow();

  public:

  WindowSelector(const PointCloudT::Ptr input);
  ~WindowSelector();
  void spin();
  vector<PointCloudWindow> faces();
};

#endif
