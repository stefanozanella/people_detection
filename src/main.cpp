#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;

using pcl::io::loadPCDFile;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "Usage: " << argv[0] << " <point_cloud_file>" << endl;
    exit(1);
  }

  PointCloudT::Ptr cloud (new PointCloudT);

  if (loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    PCL_ERROR("Couldn't read the pcd file.\n");
    return -1;
  }

  cout << "Original point cloud: " << cloud->width << " x " << cloud->height << endl;
  cout << "Point cloud size: " << cloud->width * cloud->height << endl;

  int window_width = 64;
  int window_height = 64;
  int step_size = 64;

  //int k = 30, j = 264; // Face 1 img 25
  //int k = 100, j = 300; // Face 2 img 25
  //int k = 80, j = 350; // Face 3 img 25
  // int k = 70, j = 450; // Face 4 img 25
  for (int k = 0; k < cloud->height - window_height; k += step_size) {
    for (int j = 0; j < cloud->width - window_width; j += step_size) {
      //int x_from = j, x_to = j + window_width, y_from = k, y_to = k + window_height;
      //cout << "Extracting window (" << x_from << " - " << x_to << ") x ("<< y_from << " - " << y_to << ")" << endl;

      PointCloudT::Ptr window (new PointCloudT);

      // TODO Why does it work while instead if I copy the range of points
      // manually the colors are screwed up? I think the answer lies in either
      // pcl::ExtractIndices.filter or pcl::PCLBase.setIndices.
      // Solving this (i.e. being able to do this manually) would probably allow
      // for a great speedup, as each new window can be built by removing a
      // row/column and adding another one.
      // Q: Will all this matter since most of the work is done via an integral
      // image?
      pcl::ExtractIndices<PointT> ei (false);
      ei.setInputCloud(cloud);
      ei.setIndices(k, j, window_height, window_width);
      ei.setKeepOrganized(true);
      ei.filter(*window);

      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setCameraPosition(0,0,-2,0,-1,0,0);

      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(window);
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_c(cloud);

      int left_vp, right_vp;
      viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left_vp);
      viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right_vp);

      viewer.addPointCloud<PointT> (cloud, rgb_c, "cloud", left_vp);
      viewer.addPointCloud<PointT> (window, rgb, "window", right_vp);
      viewer.spin();
    }
  }

  return 0;
}
