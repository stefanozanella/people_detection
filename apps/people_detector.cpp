/**
 * GOAL:
 * [x] load detector
 * [x] "sliding window"
 * [~] detect face on each sub-window
 * [ ] visualize bounding box on detected faces
 */
#include <string>
#include <iostream>
#include <cmath>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "strong_classifier.h"
#include "load_trained_detector.h"
#include "sub_window.h"

#include "training_sample.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;
using pcl::io::loadPCDFile;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void find_people(const PointCloudT::Ptr sample, const StrongClassifier& detector) {
  SubWindow sub_window (sample);

  int x = 0, y = 0, current_win_size;
  const int base_win_size = 148; // TODO

  int pos = 0, neg = 0;
  vector<Rect> faces;

  while (y < sample->height) {
    while (x < sample->width) {
      float center = sample->at(x, y).z;

      if (pcl_isnan(center)) {
        x++;
        continue;
      }

      current_win_size = ceil(base_win_size / sample->at(x, y).z);

      int x_from = x - floor(current_win_size / 2.0);
      int x_to = x + ceil(current_win_size / 2.0);
      int y_from = y - floor(current_win_size / 2.0);
      int y_to = y + ceil(current_win_size / 2.0);

      if (x_from >= 0 && x_to < sample->width && y_from >= 0 && y_to < sample->height) {
        sub_window.crop(x_from, y_from, current_win_size);

        if (detector.is_face(sub_window)) {
          faces.push_back(Rect(x_from, y_from, x_to - x_from, y_to - y_from, 1));
          pos++;
        } else {
          neg++;
        }
      }

      x++; // TODO Maybe it's possible to move forward faster?
    }

    y++; // TODO Maybe it's possible to move forward faster?
    x = 0;
  }

  cout << "Positive: " << pos << " - Negative: " << neg << endl;

  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(sample);

  viewer.addPointCloud<PointT>(
    sample,
    rgb,
    "sample"
  );

  viewer.spin();
}

int main(int argc, char** argv) {
  StrongClassifier detector;
  if (!load_trained_detector("face_detector.yml", detector)) {
    cout << "Couldn't load classifier" << endl;
    return -1;
  }

  directory_iterator sample ((path(string(argv[1]))));
  directory_iterator no_more_samples;

  for (; sample != no_more_samples; sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(sample->path().string(), *(sample_cloud)) == -1) {
      cout << "Couldn't load cloud file " << sample->path() << endl;
      return -1;
    }

    find_people(sample_cloud, detector);

    //PointCloudT::Ptr test_cloud (new PointCloudT);
    //ExtractIndices extractor(false);
    //extractor.setInputCloud(sample_cloud);
    //extractor.setIndices(101, 201, 45, 45);
    //extractor.filter(*test_cloud);
    //test_cloud->width = test_cloud->height = 45;
    //std::cout << test_cloud->at(0, 0) << std::endl;
    //std::cout << test_cloud->at(test_cloud->height-1, 0) << std::endl;
    //std::cout << test_cloud->at(0, test_cloud->width-1) << std::endl;
    //std::cout << test_cloud->at(test_cloud->height-1, test_cloud->width-1) << std::endl;
    //TrainingSample test (test_cloud, true);
    //SubWindow sw (sample_cloud);
    //sw.crop(101, 201, 45);
  }

  return 0;
}

//#include <pcl/filters/extract_indices.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//typedef pcl::PointXYZI MonochromePointT;
//typedef pcl::PointCloud<MonochromePointT> MonochromePointCloudT;
//
//int main(int argc, char** argv) {
//  if (argc < 2) {
//    cout << "Usage: " << argv[0] << " <point_cloud_file>" << endl;
//    exit(1);
//  }
//
//  /////////////////////////////////////////////////////////////////////////////
//  // Sliding window
//  /////////////////////////////////////////////////////////////////////////////
//
//  int window_width = 64;
//  int window_height = 64;
//  int step_size = 64;
//
//  //int k = 30, j = 264; // Face 1 img 25
//  //int k = 100, j = 300; // Face 2 img 25
//  //int k = 80, j = 350; // Face 3 img 25
//  // int k = 70, j = 450; // Face 4 img 25
//  for (int k = 0; k < cloud->height - window_height; k += step_size) {
//    for (int j = 0; j < cloud->width - window_width; j += step_size) {
//      //int x_from = j, x_to = j + window_width, y_from = k, y_to = k + window_height;
//      //cout << "Extracting window (" << x_from << " - " << x_to << ") x ("<< y_from << " - " << y_to << ")" << endl;
//
//      PointCloudT::Ptr window (new PointCloudT);
//
//      // TODO Manyually copy the window's points, setting is_dense to false in
//      // the window cloud (or finding a way to remove the NaN while keeping the
//      // point cloud organized).
//      // Doin this would probably allow
//      // for a great speedup, as each new window can be built by removing a
//      // row/column and adding another one.
//      // Q: Will all this matter since most of the work is done via an integral
//      // image?
//      pcl::ExtractIndices<PointT> ei (false);
//      ei.setInputCloud(cloud);
//      ei.setIndices(k, j, window_height, window_width);
//      ei.setKeepOrganized(true);
//      ei.filter(*window);
//
//      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//      viewer.setCameraPosition(0,0,-2,0,-1,0,0);
//
//      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(window);
//      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_c(cloud);
//
//      int left_vp, right_vp;
//      viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left_vp);
//      viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right_vp);
//
//      viewer.addPointCloud<PointT> (cloud, rgb_c, "cloud", left_vp);
//      viewer.addPointCloud<PointT> (window, rgb, "window", right_vp);
//      viewer.spin();
//    }
//  }
//
//  return 0;
//}
