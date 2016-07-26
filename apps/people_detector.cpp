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
 * [x] detection clustering
 *   [x] check min/max boundaries, check why it doesn't cluster properly -> you
 *   have to set the "from" variable to INT_MAX otherwise they'll always be 0 ->
 *   not much difference from the average, anyway
 *   [x] try to limit the condition of clustering: set a 10px radius around the
 *   center rather than the whole rectangle -> a 14px radius seems to be better
 *   for now, but it can probably change if the quality of the detector
 *   improves
 * [ ] improve shifting process
 * [?] find position of detection, bypass the face detection process, work on
 * body detection
 *   [ ] separate face from background/adjacent objects
 *     [x] find a way to use the detection made in the original cloud to extract
 *     the equivalent window in a cloud filtered by a voxel grid (to eliminate
 *     NaNs and be able to use, e.g., EuclideanClusterExtraction) -> maybe use
 *     http://docs.pointclouds.org/trunk/classpcl_1_1octree_1_1_octree_point_cloud_search.html#a6f3beb07d39a121bd3a8b450dd972da3
 *     [x] refactor find_people so that it just returns an array of detected
 *     faces (rename to find_faces?)
 *     [x] apply voxel grid over whole cloud
 *     [x] for each face detection
 *       [x] find min/max boundaries on the face detection
 *       [x] apply box search on filtered cloud
 *       [x] try euclidean distance clustering + selection of cluster with max
 *       amount of points as detected face ->
 *       -> I think this approach showed some interesting points, but the
 *       euclidean clustering algorithm doesn't seem to perform very well.
 *       Ideas:
 *         [-] tune euclidean clustering parameters
 *         [x] explore other segmentation approaches -> region growing seems to work quite well if tuned correctly
 *           [x] test region growing with other samples, see how it fares -> seems to work quite well, maybe some params can be tuned down a bit
 *        [~] implement a variant of region growing that only develops a region
 *        starting from a seed. Start from the PCL implementation of region
 *        growing, use the centroid of the detection and see if we manage to
 *        cover the whole body. -> found RegionGrowing::getSegmentFromPoint(),
 *        but it still does a full scan to perform the segmentation, then doing
 *        another full scan to find the segment to which the point belongs to.
 *          [ ] improve region growing algorithm so that starting from the seed
 *          only one region is grown out of the seed
 *          [ ] improve scaffolding code to avoid extracting the region of the
 *          face and then performing another region growing over it. The problem
 *          here was that picking the center of the window in the sub-sampled
 *          cloud almost never picks up a point from the face. Not sure how to
 *          solve it but I have the feeling that the OcTree::boxSearch is
 *          returning something weird.
 *        [-] once the face region has been segmented, try to use octree radiusSearch to
 *        iteratively expand the region of interest. Probably there's a need
 *        of finding some sort of algorithm that decides when to stop,
 *        to make the whole tolerant to figures next to a background object
 *        [-] another approach could be to iteratively search for body portions:
 *        first, given the head we can look for the torso in a pre-defined box
 *        area around the face; next, we can try to look for arms and legs in
 *        predefined areas around the torso (i.e. given a torso, an arm can
 *        start only from a specific point - more or less, has a definite
 *        max length and can only move in specific directions)
 */
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
#include "pcl/segmentation/single_seed_region_growing.h"

#include "cascade_classifier.h"
#include "load_trained_detector.h"
#include "sub_window.h"

#include "training_sample.h"

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
typedef pcl::SingleSeedRegionGrowing<PointT, pcl::Normal> SingleSeedRegionGrowing;

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

void find_faces(const PointCloudT::Ptr sample, const CascadeClassifier& detector, vector<Face>& faces) {
  SubWindow sub_window (sample);

  //int x = 200, y = 200, current_win_size;
  int x = 0, y = 0, current_win_size;
  const int base_win_size = 148; // TODO

  int pos = 0, neg = 0;
  vector<vector<Rect>*> detection_buckets;
  vector<Rect>* detection_map [sample->width][sample->height];

  while (y < sample->height) {
    while (x < sample->width) {
  //while (y < 260) {
  //  while (x < 260) {
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

          //for (int j = x-7; j <= x+7; j++) {
          //  for (int k = y-7; k <= y+7; k++) {
          //    detection_map[j][k] = current_bucket;
          //  }
          //}

          pos++;
        } else {
          neg++;
        }
      }

      x+=1; // TODO Maybe it's possible to move forward faster?
    }

    y+=1; // TODO Maybe it's possible to move forward faster?
    //x = 200;
    x = 0;
  }

  cout << "Pos: " << pos << ", Neg: " << neg << endl;

  for (vector<vector<Rect>*>::iterator bucket = detection_buckets.begin(); bucket != detection_buckets.end(); bucket++) {
    int cluster_from_y = 0, cluster_from_x = 0, cluster_to_x = 0, cluster_to_y = 0;

    for (vector<Rect>::iterator face = (*bucket)->begin(); face != (*bucket)->end(); face++) {
      cluster_from_y += face->y;
      cluster_from_x += face->x;
      cluster_to_x += face->x + face->width;
      cluster_to_y += face->y + face->height;
    }

    cluster_from_y /= (*bucket)->size();
    cluster_from_x /= (*bucket)->size();
    cluster_to_x /= (*bucket)->size();
    cluster_to_y /= (*bucket)->size();

    PointXYZ face_min_boundary, face_max_boundary;
    bounded_min_max(sample, cluster_from_x, cluster_from_y, cluster_to_x, cluster_to_y, face_min_boundary, face_max_boundary);

    faces.push_back(
      Face(
        face_min_boundary,
        face_max_boundary,
        sample->at(
          (cluster_from_x + cluster_to_x) / 2,
          (cluster_from_y + cluster_to_y) / 2
        )
      )
    );
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

  OctreeSearch octree (32.0f);
  octree.setInputCloud(sample);
  octree.addPointsFromInputCloud();

  int k;
  vector<Face>::iterator face;
  for (k = 0, face = faces.begin(); face != faces.end(); k++, face++) {
    vector<int> indices;
    octree.boxSearch(
      Eigen::Vector3f(face->min_boundary.x, face->min_boundary.y, face->min_boundary.z),
      Eigen::Vector3f(face->max_boundary.x, face->max_boundary.y, face->max_boundary.z),
      indices
    );

    PointCloudT::Ptr face_cloud (new PointCloudT);

    for (vector<int>::iterator index = indices.begin(); index != indices.end(); index++) {
      PointT p = sample->at(*index);
      p.r = 255;
      p.g = p.b = 0;
      face_cloud->points.push_back(p);
    }

    viewer.addPointCloud<PointT>(
      face_cloud,
      pcl::visualization::PointCloudColorHandlerRGBField<PointT>(face_cloud),
      "face"+boost::to_string(k)
    );

    viewer.spin();
    viewer.removeAllShapes();
    viewer.removePointCloud("face"+boost::to_string(k));
  }
}

void find_bodies(PointCloudT::Ptr& cloud, vector<Face>& faces) {
  // TODO Doesn't belong here, remove
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  viewer.addPointCloud<PointT>(
    cloud,
    pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cloud),
    "cloud"
  );
  // TODO Doesn't belong here, remove


  OctreeSearch octree (32.0f);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  KdTree::Ptr tree (new KdTree);
  tree->setInputCloud(cloud);

  NormalCloudT::Ptr normals (new NormalCloudT);
  NormalEstimation normal_estimation;
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setKSearch(50);
  normal_estimation.compute(*normals);

  SingleSeedRegionGrowing reg;
  reg.setMinClusterSize(200);
  reg.setMaxClusterSize(1000000);
  reg.setNumberOfNeighbours(60);
  reg.setSmoothnessThreshold(15.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  reg.setSearchMethod(tree);
  reg.setInputCloud(cloud);
  reg.setInputNormals(normals);

  int j = 0;
  for (vector<Face>::iterator face = faces.begin(); face != faces.end(); face++) {
    // TODO
    // - compute face centroid
    // - do a k nearest search for k = 1
    // - use selected point as seed for single seed region growing
    vector<int> indices;
    octree.boxSearch(
      Eigen::Vector3f(face->min_boundary.x, face->min_boundary.y, face->min_boundary.z),
      Eigen::Vector3f(face->max_boundary.x, face->max_boundary.y, face->max_boundary.z),
      indices
    );

    reg.setIndices(boost::shared_ptr<vector<int> >(new vector<int>(indices)));

    // TODO Just trying out stuff, redo properly
    vector<pcl::PointIndices> cluster_indices;
    reg.extract(cluster_indices);

    PointCloudT::Ptr face_cloud (new PointCloudT);

    for (vector<int>::iterator index = indices.begin(); index != indices.end(); index++) {
      PointT p = cloud->at(*index);
      p.r = 255;
      p.g = p.b = 0;
      face_cloud->points.push_back(p);
    }

    viewer.addPointCloud<PointT>(
      face_cloud,
      pcl::visualization::PointCloudColorHandlerRGBField<PointT>(face_cloud),
      "face"+boost::to_string(j)
    );
    int k = 0;
    const vector<int>* biggest_cluster = &(cluster_indices[0].indices);
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      if (it->indices.size() > biggest_cluster->size())
        biggest_cluster = &(it->indices);
    }

    pcl::PointIndices body_indices;
    SingleSeedRegionGrowing r;
    r.setMinClusterSize(200);
    r.setMaxClusterSize(1000000);
    r.setNumberOfNeighbours(60);
    r.setSmoothnessThreshold(15.0 / 180.0 * M_PI);
    r.setCurvatureThreshold(1.0);
    r.setSearchMethod(tree);
    r.setInputCloud(cloud);
    r.setInputNormals(normals);
    r.getSegmentFromPoint(biggest_cluster->at(biggest_cluster->size() / 2), body_indices);

    PointCloudT::Ptr cluster_cloud (new PointCloudT);
    for (vector<int>::const_iterator pit = body_indices.indices.begin(); pit != body_indices.indices.end(); ++pit) {
      PointT p = cloud->at(*pit);
      p.b = 255;
      p.r = p.g = 0;
      cluster_cloud->points.push_back(p);
    }

    viewer.addPointCloud<PointT>(
        cluster_cloud,
        pcl::visualization::PointCloudColorHandlerRGBField<PointT>(cluster_cloud),
        "cluster"+boost::to_string(k)
        );

    viewer.spin();
    viewer.removePointCloud("cluster"+boost::to_string(k));
    k++;
    viewer.removePointCloud("face"+boost::to_string(j));
    j++;
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
      cout << "Couldn't load cloud file " << sample->path() << endl;
      return -1;
    }

    vector<Face> faces;
    find_faces(sample_cloud, detector, faces);

    PointCloudT::Ptr filtered_cloud (new PointCloudT);
    VoxelGrid voxel;
    voxel.setInputCloud(sample_cloud);
    voxel.setLeafSize(0.01, 0.01, 0.01);
    voxel.filter(*filtered_cloud);

    show_faces(filtered_cloud, faces);

    //find_bodies(filtered_cloud, faces);
  }

  return 0;
}
//  //int k = 30, j = 264; // Face 1 img 25
//  //int k = 100, j = 300; // Face 2 img 25
//  //int k = 80, j = 350; // Face 3 img 25
//  // int k = 70, j = 450; // Face 4 img 25
