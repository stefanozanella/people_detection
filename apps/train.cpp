/**
 * GOAL:
 * [x] load samples
 * [ ] calculate samples's integral image
 * [ ] generate features
 * [ ] select first weak classifier
 */
#include <string>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include "training_sample.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using pcl::io::loadPCDFile;

int main(int argc, char** argv) {
  string positive_samples_dir (argv[1]), negative_samples_dir (argv[2]);

  directory_iterator positive_sample ((path(positive_samples_dir)));
  directory_iterator negative_sample ((path(negative_samples_dir)));
  directory_iterator no_more_samples;

  vector<TrainingSample> samples;

  for (; positive_sample != no_more_samples; positive_sample++) {
    TrainingSample sample (true);

    if (loadPCDFile<PointT>(positive_sample->path().string(), *(sample.cloud)) == -1) {
      cout << "Couldn't load cloud file " << positive_sample->path() << endl;
      return -1;
    }

    samples.push_back(sample);
  }

  for (; negative_sample != no_more_samples; negative_sample++) {
    TrainingSample sample (false);

    if (loadPCDFile<PointT>(negative_sample->path().string(), *(sample.cloud)) == -1) {
      cout << "Couldn't load cloud file " << negative_sample->path() << endl;
      return -1;
    }

    samples.push_back(sample);
  }

  cout << samples.size() << endl;

  return 0;
}
