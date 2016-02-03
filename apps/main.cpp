#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;

using pcl::io::loadPCDFile;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZI MonochromePointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<MonochromePointT> MonochromePointCloudT;

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

  /////////////////////////////////////////////////////////////////////////////
  // Integral image
  /////////////////////////////////////////////////////////////////////////////

  /**
   * Calculates integral image. At a specific location (j,k), the integral image
   * can be computed as the algebraic addition the follows:
   * +-------+---+
   * |       |   |
   * |   A   | B |
   * |       |   |
   * +-------+---+
   * |   C   | D |
   * +-------+---+(j,k)
   *
   * ii(j,k) = D + B + C - A
   *
   * Where:
   *
   * D = cloud[j,k]
   * B = ii(j-1, k)
   * C = ii(j, k-1)
   * A = ii(j-1, k-1)
   *
   * On the 1st row and 1st column, we must subsititute values where one or both
   * indexes are out of range with the value 0 (i.e. the particular element is
   * not to be considered). The same effect can be obtained by augmenting the
   * image with a row and a column of 0s in the first position:
   *
   * 0|00000000000|
   * -+-------+---+
   * 0|       |   |
   * 0|   A   | B |
   * 0|       |   |
   * -+-------+---+
   * 0|   C   | D |
   * -+-------+---+(j,k)
   */

  MonochromePointCloudT::Ptr integral_image (new MonochromePointCloudT);

  integral_image->width = cloud->width;
  integral_image->height = cloud->height;
  integral_image->points.resize(cloud->points.size());
  integral_image->is_dense = false;

  for (int k = 0; k < cloud->height; k++) {
    for (int j = 0; j < cloud->width; j++) {
      // TODO What if 0,0 is NaN? What if another point is Nan?
      // TODO A: I don't care because all that matters are intensities for now.
      // TODO Hint: colors might be NaN too.
      MonochromePointT d;
      pcl::PointXYZRGBtoXYZI(cloud->at(j,k), d);

      float a = j < 1 || k < 1 ? 0 : integral_image->at(j-1, k-1).intensity;
      float b = j < 1 ? 0 : integral_image->at(j-1, k).intensity;
      float c = k < 1 ? 0 : integral_image->at(j, k-1).intensity;

      d.intensity += b + c - a;
      integral_image->at(j, k) = d;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // DEBUG
  /////////////////////////////////////////////////////////////////////////////

  pcl::visualization::PCLVisualizer monoviewer("PCL Viewer");
  monoviewer.setCameraPosition(0,0,-2,0,-1,0,0);

  pcl::visualization::PointCloudColorHandlerGenericField<MonochromePointT> mono(integral_image, "intensity");
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb1(cloud);

  int left_vp1, right_vp1;
  monoviewer.createViewPort(0.0, 0.0, 0.5, 1.0, left_vp1);
  monoviewer.createViewPort(0.5, 0.0, 1.0, 1.0, right_vp1);

  monoviewer.addPointCloud<PointT> (cloud, rgb1, "cloud", left_vp1);
  monoviewer.addPointCloud<MonochromePointT> (integral_image, mono, "window", right_vp1);
  monoviewer.spin();

  return 0;

  /////////////////////////////////////////////////////////////////////////////
  // Sliding window
  /////////////////////////////////////////////////////////////////////////////

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

      // TODO Manyually copy the window's points, setting is_dense to false in
      // the window cloud (or finding a way to remove the NaN while keeping the
      // point cloud organized).
      // Doin this would probably allow
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
