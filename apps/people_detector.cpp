#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl/segmentation/single_seed_region_growing_rgb.h"
#include "cascade_classifier.h"
#include "load_trained_detector.h"
#include "sub_window.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;
using pcl::io::loadPCDFile;
using pcl::PointXYZ;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::ExtractIndices<PointT> ExtractIndices;
typedef pcl::VoxelGrid<PointT> VoxelGrid;
typedef pcl::octree::OctreePointCloudSearch<PointT> OctreeSearch;
typedef pcl::search::KdTree<PointT> KdTree;
typedef pcl::PointCloud<pcl::Normal> NormalCloudT;
typedef pcl::NormalEstimation<PointT, pcl::Normal> NormalEstimation;
typedef pcl::SingleSeedRegionGrowingRGB<PointT, pcl::Normal> RegionGrowingRGB;

typedef struct Face {
  PointXYZ min_boundary, max_boundary;
  PointT center;
  Face(PointXYZ min, PointXYZ max, PointT center) : min_boundary (min), max_boundary (max), center (center) {}
} Face;

void bounded_min_max(PointCloudT::Ptr sample, int from_x, int from_y, int to_x, int to_y, PointXYZ& min, PointXYZ& max) {
  min.x = min.y = min.z = FLT_MAX;
  max.x = max.y = max.z = FLT_MIN;

  for (int j = from_x; j <= to_x; j++) {
    for (int k = from_y; k <= to_y; k++) {
      min.x = std::min(min.x, sample->at(j,k).x);
      min.y = std::min(min.y, sample->at(j,k).y);
      min.z = std::min(min.z, sample->at(j,k).z);

      max.x = std::max(max.x, sample->at(j,k).x);
      max.y = std::max(max.y, sample->at(j,k).y);
      max.z = std::max(max.z, sample->at(j,k).z);
    }
  }
}

void show_faces(PointCloudT::Ptr& sample, vector<Face>& faces) {
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  viewer.addPointCloud<PointT>(
    sample,
    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(sample),
    "sample"
  );

  for (int k = 0; k < faces.size(); k++) {
    viewer.addCube(
      faces.at(k).min_boundary.x,
      faces.at(k).max_boundary.x,
      faces.at(k).min_boundary.y,
      faces.at(k).max_boundary.y,
      faces.at(k).min_boundary.z,
      faces.at(k).max_boundary.z,
      0, 1, 0,
      "face"+boost::to_string(k)
    );
  }

  viewer.spin();
}

bool is_the_size_of_a_face(const PointCloudT::Ptr sample, int x_from, int y_from, int x_to, int y_to) {
  PointXYZ min_boundary, max_boundary;
  bounded_min_max(
    sample,
    x_from,
    y_from,
    x_to,
    y_to,
    min_boundary,
    max_boundary
  );

  float distance = sqrt(
    pow(max_boundary.x - min_boundary.x, 2) +
    pow(max_boundary.y - min_boundary.y, 2) +
    pow(max_boundary.z - min_boundary.z, 2)
  );

  return distance > 0.6 && distance < 0.8;
}

void find_faces(const PointCloudT::Ptr sample, const CascadeClassifier& detector, vector<Face>& faces) {
  SubWindow sub_window (sample);

  int x = 0, y = 0, current_win_size;
  const int base_win_size = 148;

  int pos = 0, neg = 0;
  vector<vector<Rect>*> detection_buckets;
  vector<Rect>* detection_map [sample->width][sample->height];

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

        if (detector.is_face(sub_window) && is_the_size_of_a_face(sample, x_from, y_from, x_to, y_to)) {
          pos++;

          PointXYZ face_min_boundary, face_max_boundary;
          bounded_min_max(
            sample,
            x_from,
            y_from,
            x_to,
            y_to,
            face_min_boundary,
            face_max_boundary
          );

          faces.push_back(
            Face(
              face_min_boundary,
              face_max_boundary,
              sample->at(
                (x_from + x_to) / 2,
                (y_from + y_to) / 2
              )
            )
          );
        } else {
          neg++;
        }
      }

      x+=5;
    }

    y+=5;
    x = 0;
  }

  cout << "Pos: " << pos << ", Neg: " << neg << endl;
}

void expand_faces_to_bodies(const PointCloudT::Ptr sample, vector<Face>& faces, vector<pcl::PointIndices::Ptr>& bodies) {
  KdTree::Ptr tree = KdTree::Ptr (new KdTree);
  tree->setInputCloud(sample);

  RegionGrowingRGB reg;
  reg.setInputCloud(sample);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(10);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(600);

  for (vector<Face>::iterator face = faces.begin(); face != faces.end(); face++) {
    vector<int> closest_index (1);
    vector<float> closest_distance (1);

    tree->nearestKSearch(face->center, 1, closest_index, closest_distance);

    pcl::PointIndices::Ptr body (new pcl::PointIndices);
    reg.getSegmentFromPoint(closest_index.at(0), *body);

    if (body->indices.size() > 2000) {
      bodies.push_back(body);
    }
  }
}

int main(int argc, char** argv) {
  CascadeClassifier detector;

  if (!load_trained_detector("face_detector.yml", detector)) {
    cout << "Couldn't load classifier" << endl;
    return -1;
  }

  directory_iterator sample ((path(string(argv[1]))));
  directory_iterator no_more_samples;

  for (; sample != no_more_samples; sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(sample->path().string(), *(sample_cloud)) == -1) {
      cout << "Couldn't load cloud file " << argv[1] << endl;
      return -1;
    }

    vector<Face> faces;
    vector<pcl::PointIndices::Ptr> bodies;

    find_faces(sample_cloud, detector, faces);

    PointCloudT::Ptr filtered_sample = PointCloudT::Ptr (new PointCloudT);

    VoxelGrid voxel;
    voxel.setInputCloud(sample_cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel.filter(*filtered_sample);

    PointCloudT::Ptr plane_detection_cloud (new PointCloudT);
    copyPointCloud(*filtered_sample, *plane_detection_cloud);

    Eigen::Vector4f ground_plane_centroid;
    pcl::PointIndices::Ptr ground_plane_indices;

    ground_plane_centroid[2] = FLT_MAX;

    while (true) {
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(1000);
      seg.setDistanceThreshold(0.1);
      seg.setAxis(Eigen::Vector3f(1,0,0));
      seg.setEpsAngle(0.05);

      seg.setInputCloud(plane_detection_cloud);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size () == 0) {
        break;
      }

      ExtractIndices ei;
      ei.setInputCloud(plane_detection_cloud);
      ei.setIndices(inliers);
      ei.setNegative(true);

      PointCloudT::Ptr filtered_cloud_no_ground_plane (new PointCloudT);
      ei.filter(*filtered_cloud_no_ground_plane);

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*filtered_sample, *inliers, centroid);

      if (centroid[2] < ground_plane_centroid[2] && inliers->indices.size() > 20000) {
        ground_plane_centroid = centroid;
        ground_plane_indices = inliers;
      }

      plane_detection_cloud.swap(filtered_cloud_no_ground_plane);
    }

    ExtractIndices ei;
    ei.setInputCloud(filtered_sample);
    ei.setIndices(ground_plane_indices);
    ei.setNegative(true);

    PointCloudT::Ptr filtered_cloud_no_ground_plane (new PointCloudT);
    ei.filter(*filtered_cloud_no_ground_plane);

    expand_faces_to_bodies(filtered_cloud_no_ground_plane, faces, bodies);

    cout << "Total clusters found: " << bodies.size() << endl;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    for (int k = 0; k < bodies.size(); k++) {
      ExtractIndices ei;
      ei.setInputCloud(filtered_cloud_no_ground_plane);
      ei.setIndices(bodies.at(k));

      PointCloudT::Ptr body_cloud (new PointCloudT);
      ei.filter(*body_cloud);

      viewer.addPointCloud<PointT>(
          body_cloud,
          pcl::visualization::PointCloudColorHandlerRGBField<PointT>(body_cloud),
          "body"+boost::to_string(k)
          );

    }

    viewer.spin();
    //show_faces(sample_cloud, faces);
  }

  return 0;
}
