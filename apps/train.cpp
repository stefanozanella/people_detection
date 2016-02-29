/**
 * GOAL:
 * [x] load samples
 * [x] calculate samples's integral image
 * [x] generate features
 * [x] verify generated features and feature calculation
 * [ ] select first weak classifier
 */
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include "training_sample.h"
#include "rect.h"
#include "feature.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using pcl::io::loadPCDFile;
using std::max;

int main(int argc, char** argv) {
  string positive_samples_dir (argv[1]), negative_samples_dir (argv[2]);

  directory_iterator positive_sample ((path(positive_samples_dir)));
  directory_iterator negative_sample ((path(negative_samples_dir)));
  directory_iterator no_more_samples;

  vector<TrainingSample> samples;

  for (; positive_sample != no_more_samples; positive_sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(positive_sample->path().string(), *(sample_cloud)) == -1) {
      cout << "Couldn't load cloud file " << positive_sample->path() << endl;
      return -1;
    }

    samples.push_back(TrainingSample(sample_cloud, true));
  }

  for (; negative_sample != no_more_samples; negative_sample++) {
    PointCloudT::Ptr sample_cloud (new PointCloudT);

    if (loadPCDFile<PointT>(negative_sample->path().string(), *(sample_cloud)) == -1) {
      cout << "Couldn't load cloud file " << negative_sample->path() << endl;
      return -1;
    }

    samples.push_back(TrainingSample(sample_cloud, false));
  }

  uint32_t base_win_size = 0;
  // Invariants:
  // - samples are square in size (width == height)
  // - samples are organized
  for (vector<TrainingSample>::iterator sample = samples.begin(); sample != samples.end(); sample++) {
    base_win_size = max(base_win_size, sample->cloud->width);
  }

  cout << "Base win size for the training set: " << base_win_size << endl;

  int count = 0;

  vector<Feature> features;

  for (uint32_t x = 0; x < base_win_size; x++) {
    for (uint32_t y = 0; y < base_win_size; y++) {
      // generate horizontal haar2
      for (uint32_t dx = 1; 2*dx < base_win_size - x; dx++) {
        for (uint32_t dy = 1; dy < base_win_size - y; dy++) {
          features.push_back(
            Feature() <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1)
          );
          count++;
        }
      }

      // generate vertical haar2
      for (int dx = 1; dx < base_win_size - x; dx++) {
        for (int dy = 1; 2*dy < base_win_size - y; dy++) {
          features.push_back(
            Feature() <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x, y + dy, dx, dy, 1)
          );
          count++;
        }
      }

      // generate horizontal haar3
      for (int dx = 1; 3*dx < base_win_size - x; dx++) {
        for (int dy = 1; dy < base_win_size - y; dy++) {
          features.push_back(
            Feature() <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1) <<
            Rect(x + 2*dx, y, dx, dy, -1)
          );
          count++;
        }
      }

      // generate vertical haar3
      for (int dx = 1; dx < base_win_size - x; dx++) {
        for (int dy = 1; 3*dy < base_win_size - y; dy++) {
          features.push_back(
            Feature() <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x, y + dy, dx, dy, 1) <<
            Rect(x, y + 2*dy, dx, dy, -1)
          );
          count++;
        }
      }

      // generate haar4
      for (int dx = 1; 4*dx < base_win_size - x; dx++) {
        for (int dy = 1; 4*dy < base_win_size - y; dy++) {
          features.push_back(
            Feature() <<
            Rect(x, y, dx, dy, -1) <<
            Rect(x + dx, y, dx, dy, 1) <<
            Rect(x, y + dy, dx, dy, 1) <<
            Rect(x + dx, y + dy, dx, dy, -1)
          );
          count++;
        }
      }
    }
  }

  cout << "Generated " << count << " features" << endl;

  //////////////////////////////////////
  // JUST TESTING
  //////////////////////////////////////
  Feature f = features.at(16060800);

  for (vector<Rect>::iterator it = f.rectangles.begin(); it != f.rectangles.end(); it++) {
    cout << "x: " << it->x << " y: " << it->y << " w: " << it->width << " h: " << it->height << " m: " << it->multiplier << endl;
  }

  PointCloudT::Ptr test_cloud (new PointCloudT);
  test_cloud->width = test_cloud->height = base_win_size;
  test_cloud->is_dense = true;

  for (int y = 0; y < test_cloud->height; y++) {
    for (int x = 0; x < test_cloud->width; x++) {
      PointT p;

      p.x = x;
      p.y = y;
      p.z = 2;

      if (x < 35 || y < 10 || x > 60 || y > 23) {
        p.r = p.g = p.b = 1;
      }

      // top-left
      if (34 < x && x < 48 && 9 < y && y < 17) {
        p.r = p.g = p.b = 0;
        //p.r = p.g = p.b = (x-35)%13 + 13*(y-10);
      }

      // top-right
      if (47 < x && x < 61 && 9 < y && y < 17) {
        //p.r = p.g = p.b = 0;
        p.r = p.g = p.b = (x-48)%13 + 13*(y-10);
      }

      // bottom-left
      if (34 < x && x < 48 && 16 < y && y < 24) {
        //p.r = p.g = p.b = 0;
        p.r = p.g = p.b = (x-35)%13 + 13*(y-17);
      }

      // bottom-right
      if (47 < x && x < 61 && 16 < y && y < 24) {
        p.r = p.g = p.b = 0;
        //p.r = p.g = p.b = (x-48)%13 + 13*(y-17);
      }

      test_cloud->points.push_back(p);
    }
  }

  TrainingSample test_sample (test_cloud, true);

  float feature_value;

  for (vector<Rect>::iterator it = f.rectangles.begin(); it != f.rectangles.end(); it++) {
    feature_value += it->multiplier * test_sample.integral_sum(it->x, it->y, it->x + it->width - 1, it->y + it->height - 1);
  }

  cout << "Feature value: " << feature_value << endl;

  return 0;
}
