#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::io::loadPCDFile;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> RGBHandler;
typedef pcl::ExtractIndices<PointT> ExtractIndices;

int main(int argc, char** argv) {
  int win_size = 64;

  PointCloudT::Ptr cloud (new PointCloudT);
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

  viewer.spin();

  return 0;
}
