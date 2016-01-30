#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::io::loadPCDFile;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> RGBHandler;
typedef pcl::ExtractIndices<PointT> ExtractIndices;
typedef pcl::visualization::PointPickingEvent PointPickingEvent;
typedef pcl::search::KdTree<PointT> KdTree;

int win_size = 64;

PointCloudT::Ptr cloud (new PointCloudT);

int left, right;

void relocateWindow(const PointPickingEvent &event, void* viewer_void) {
  PCLVisualizer viewer = *static_cast<PCLVisualizer*> (viewer_void);

  std::vector<int> indices (1);
  std::vector<float> distances (1);
  int index;
  PointT picked;
  event.getPoint(picked.x, picked.y, picked.z);

  KdTree search;
  search.setInputCloud(cloud);
  search.nearestKSearch(picked, 1, indices, distances);

  index = indices[0];

  int x = index / cloud->width, y = index % cloud -> width;

  PointCloudT::Ptr window (new PointCloudT);
  ExtractIndices ei (false);
  ei.setInputCloud(cloud);
  ei.setIndices(x, y, win_size, win_size);
  ei.filter(*window);

  viewer.updatePointCloud<PointT>(
      window,
      RGBHandler(window),
      "window");
}

int main(int argc, char** argv) {
  PointCloudT::Ptr window (new PointCloudT);

  if (loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    cout << "Couldn't load cloud file " << argv[1] << endl;
    return -1;
  }

  ExtractIndices ei (false);
  ei.setInputCloud(cloud);
  ei.setIndices(0, 0, win_size, win_size);
  ei.filter(*window);

  PCLVisualizer viewer ("PCL Viewer");

  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  int left, right;
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right);

  viewer.addPointCloud<PointT>(
      cloud,
      RGBHandler(cloud),
      "cloud",
      left);

  viewer.addPointCloud<PointT>(
      window,
      RGBHandler(window),
      "window",
      right);

  viewer.registerPointPickingCallback(relocateWindow, &viewer);

  viewer.spin();

  return 0;
}
