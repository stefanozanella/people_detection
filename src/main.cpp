#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;
using pcl::io::loadPCDFile;
using pcl::visualization::PCLVisualizer;

int main(int argc, char** argv) {
  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);

  if (loadPCDFile<PointXYZ>("../dataset/1.pcd", *cloud_in) == -1) {
    PCL_ERROR("Couldn't read the pcd file.\n");
    return -1;
  }

  cout << "Original point cloud: " << cloud_in->width << " x " << cloud_in->height << endl;

  PCLVisualizer viewer ("PCL Viewer");
  int viewport;
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewport);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(0.1);
	viewer.initCameraParameters();

	viewer.addText("Original cloud", 10, 10, "viewport_text", viewport);

  viewer.addPointCloud(
    cloud_in,
    "original",
    viewport
  );

  cout << "Visualizing..." << endl;
  viewer.spin();

  return 0;
}
