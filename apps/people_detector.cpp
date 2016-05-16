/**
 * GOAL:
 * [x] load detector
 * [x] "sliding window"
 * [x] detect face on each sub-window
 * [~] visualize bounding box on detected faces -> Expensive, is there a way to
 * do it without using ExtractIndices?
 * [x] verify behavior of strong detector on training sample vs sub window (must
 * be the same) -> Yes
 * [x] try to run one of the first detectors the training found to see if
 * results improve -> They're in fact much worse
 * beginning were different and should have been correct
 * [x] pre-filter subwindows based on skin tone
 *   [x] try skin tone filtering to check if it improves the result -> it does,
 *   but it depends on the extension of the search and I don't think the numbers
 *   are correct
 *   [x] check speed issues -> it's expensive. Just looking at the center of the
 *   window doesn't work at all.
 * [?] the doubt remains that something went wrong as the numbers at the very
 * [ ] detection clustering
 *   [ ] check min/max boundaries, check why it doesn't cluster properly
 *   [ ] try to limit the condition of clustering: set a 10px radius around the
 *   center rather than the whole rectangle
 * [ ] improve shifting process
 * [?] find position of detection, bypass the face detection process, work on
 * body detection
 */
#include <string>
#include <iostream>
#include <cmath>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

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
typedef pcl::ExtractIndices<PointT> ExtractIndices;

void find_people(const PointCloudT::Ptr sample, const StrongClassifier& detector) {
  SubWindow sub_window (sample);

  int x = 200, y = 200, current_win_size;
  const int base_win_size = 148; // TODO

  int pos = 0, neg = 0;
  vector<Rect> faces;
  vector<vector<Rect>*> detection_buckets;
  vector<Rect>* detection_map [sample->width][sample->height];

  //while (y < sample->height) {
  //  while (x < sample->width) {
  while (y < 260) {
    while (x < 260) {
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
          // since the detection is still weak, the larger the window you can scan,
          // the more confidence you can have that the clustering process works
          // correctly as you're assuring you include all the positive
          // detections that would happen around the face in the final detector.
          // Decent results can be obtained in the range 220 - 240 for now.
          // Idea: when finding a new detection, add it to a specific bucket
          // only if its center coordinate is within the boundaries of the
          // bucket. From this point of view it's like building an histogram.
          // To do this, we could keep a sparse matrix, where at each point we
          // store one (or more ?) reference(s) to the bucket(s), so that
          // accessing the right bucket becomes a matter of just accessing this
          // matrix with the coordinate of the center of the sample.
          // This has the disadvantage that the position of the bucket is fixed
          // at the beginning and doesn't get updated. On one side, this might
          // result in sub-optimal clustering as the first detection is likely
          // to be at the edge of the "real" center. On the other side, though,
          // this should prevent clustering caused by drifting.
          // On a second thought though, we could just update the matrix by
          // filling in the spots covered by the detection we're adding.
          // So:
          // - create an empty matrix of the size of the cloud
          // - for each detection d
          //   - if the bucket corresponding to the center of d is empty
          //     - add a new bucket, add d to the bucket
          //     - put a reference to the bucket in the whole area covered by d
          //   - else
          //     - add d to the bucket referenced at the detection center
          //     - update the matrixby putting a reference to the bucket in each
          //       point covered by d (<- EXPENSIVE STEP)

          vector<Rect>* current_bucket;
          if (!detection_map[x][y]) {
            current_bucket = new vector<Rect>();
            detection_buckets.push_back(current_bucket);
          } else {
            current_bucket = detection_map[x][y];
          }

          current_bucket->push_back(Rect(x_from, y_from, x_to - x_from, y_to - y_from, 1));

          for (int j = x_from; j <= x_to; j++) {
            for (int k = y_from; k <= y_to; k++) {
              detection_map[j][k] = current_bucket;
            }
          }

          faces.push_back(Rect(x_from, y_from, x_to - x_from, y_to - y_from, 1));
          pos++;
        } else {
          neg++;
        }
      }

      x+=1; // TODO Maybe it's possible to move forward faster?
    }

    y+=1; // TODO Maybe it's possible to move forward faster?
    x = 200;
  }

  cout << "Positive: " << pos << " - Negative: " << neg << endl;
  cout << "Clusters: " << detection_buckets.size() << endl;

  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(sample);

  viewer.addPointCloud<PointT>(
    sample,
    rgb,
    "sample"
  );

  for (int k = 0; k < detection_buckets.size(); k++) {
    vector<Rect>* bucket = detection_buckets.at(k);
    int cluster_from_y = 0, cluster_from_x = 0, cluster_to_x = 0, cluster_to_y = 0;

    for (int i = 0; i < bucket->size(); i++) {
      Rect face = bucket->at(i);

      PointCloudT::Ptr face_cloud (new PointCloudT);
      ExtractIndices ei (false);
      ei.setInputCloud(sample);
      ei.setIndices(face.y, face.x, face.height, face.width);
      ei.filter(*face_cloud);
      PointT min, max;
      pcl::getMinMax3D(*face_cloud, min, max);
      viewer.addCube(
          min.x,
          max.x,
          min.y,
          max.y,
          min.z,
          max.z,
          0.0,1.0,0.0,
          "bucket"+boost::to_string(i)+boost::to_string(k)
          );

      cluster_from_y += face.y;
      cluster_from_x += face.x;
      cluster_to_x += face.x + face.width;
      cluster_to_y += face.y + face.height;
    }

    cluster_from_y /= bucket->size();
    cluster_from_x /= bucket->size();
    cluster_to_x /= bucket->size();
    cluster_to_y /= bucket->size();

    PointCloudT::Ptr face_cloud (new PointCloudT);

    ExtractIndices ei (false);
    ei.setInputCloud(sample);
    ei.setIndices(cluster_from_y, cluster_from_x, cluster_to_y - cluster_from_y, cluster_to_x - cluster_from_x);
    ei.filter(*face_cloud);

    for (int j = 0; j < face_cloud->width; j++) {
      PointT& p = face_cloud->at(j);
      p.r = 255;
      p.g = p.b = 0;
    }

    viewer.addPointCloud<PointT>(
      face_cloud,
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> (face_cloud),
      "face"+boost::to_string(k)
    );
    //PointT min, max;
    //pcl::getMinMax3D(*face_cloud, min, max);
    //viewer.addCube(
    //  min.x,
    //  max.x,
    //  min.y,
    //  max.y,
    //  min.z,
    //  max.z,
    //  1.0,0.0,0.0,
    //  "cluster"+boost::to_string(k)
    //  );

    viewer.spin();
    viewer.removeAllShapes();
    viewer.removePointCloud("face"+boost::to_string(k));
  }

  //for (int k = 0; k < faces.size(); k++) {
  //  Rect face = faces.at(k);
  //  PointCloudT::Ptr face_cloud (new PointCloudT);

  //  ExtractIndices ei (false);
  //  ei.setInputCloud(sample);
  //  ei.setIndices(face.y, face.x, face.height, face.width);
  //  ei.filter(*face_cloud);
  //  PointT min, max;
  //  pcl::getMinMax3D(*face_cloud, min, max);
  //  viewer.addCube(
  //    min.x,
  //    max.x,
  //    min.y,
  //    max.y,
  //    min.z,
  //    max.z,
  //    0.0,1.0,0.0,
  //    "cube"+boost::to_string(k)
  //    );
  //}

  //viewer.spin();
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
