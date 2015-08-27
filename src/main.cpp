#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;

using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::VoxelGrid;
using pcl::io::loadPCDFile;
using pcl::visualization::PCLVisualizer;
using pcl::visualization::PointCloudColorHandlerRGBField;

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "Usage: " << argv[0] << " <point_cloud_file>" << endl;
    exit(1);
  }

  PointCloud<PointXYZRGB>::Ptr cloud_in (new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr cloud_downsampled (new PointCloud<PointXYZRGB>);

  if (loadPCDFile<PointXYZRGB>(argv[1], *cloud_in) == -1) {
    PCL_ERROR("Couldn't read the pcd file.\n");
    return -1;
  }

  cout << "Original point cloud: " << cloud_in->width << " x " << cloud_in->height << endl;

  VoxelGrid<PointXYZRGB> downsampler;
  downsampler.setInputCloud(cloud_in);
  downsampler.setLeafSize(0.05f, 0.05f, 0.05f);
  downsampler.filter(*cloud_downsampled);

  cout << "Downsampled point cloud: " << cloud_downsampled->width << " x " << cloud_downsampled->height << endl;

  PCLVisualizer viewer ("PCL Viewer");
  int viewport;
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewport);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(0.1);
	viewer.initCameraParameters();

	viewer.addText("Original cloud", 10, 10, "viewport_text", viewport);

  viewer.addPointCloud<PointXYZRGB>(
    cloud_downsampled,
    PointCloudColorHandlerRGBField<PointXYZRGB>(cloud_downsampled),
    "original",
    viewport
  );

  cout << "Visualizing..." << endl;
  viewer.spin();

  return 0;
}
