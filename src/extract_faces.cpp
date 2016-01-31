#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "window_selector.h"

using pcl::io::loadPCDFile;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> RGBHandler;
typedef pcl::ExtractIndices<PointT> ExtractIndices;
typedef pcl::visualization::PointPickingEvent PointPickingEvent;
typedef pcl::search::KdTree<PointT> KdTree;

int main(int argc, char** argv) {
  PointCloudT::Ptr cloud (new PointCloudT);

  if (loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    cout << "Couldn't load cloud file " << argv[1] << endl;
    return -1;
  }

  WindowSelector selector (cloud);
  selector.spin();

  return 0;
}
