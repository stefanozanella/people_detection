#include <stdlib.h>
#include <algorithm>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "window_selector.h"

using pcl::io::loadPCDFile;
using pcl::io::savePCDFileASCII;
using std::ostringstream;
using std::max;
using std::string;
using boost::filesystem::is_directory;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;

int firstFreeSampleNumberIn(string dir) {
  int lastUsed = 0;
  directory_iterator it ((path(dir)));
  directory_iterator end;
  while (it != end) {
    lastUsed = max(lastUsed, atoi(it->path().stem().string().c_str()));
    it++;
  }

  return ++lastUsed;
}

int main(int argc, char** argv) {
  WindowSelector selector;

  string input_dir (argv[1]);
  string output_dir (argv[2]);

  directory_iterator pcd_file ((path(input_dir)));
  directory_iterator end;

  int sample_number = firstFreeSampleNumberIn("dataset/positive");

  for (; pcd_file != end; pcd_file++) {
    if (is_directory(pcd_file->path()))
      continue;

    PointCloudT::Ptr cloud (new PointCloudT);

    if (loadPCDFile<PointT>(pcd_file->path().string(), *cloud) == -1) {
      cout << "Couldn't load cloud file " << argv[1] << endl;
      return -1;
    }

    selector.setInputCloud(cloud);
    selector.spin();

    for (int k = 0; k < selector.faces().size(); k++, sample_number++) {
      PointCloudT::Ptr face = selector.faces().at(k);

      ostringstream filename;
      filename << output_dir << "/" << sample_number << ".pcd";
      savePCDFileASCII(filename.str(), *face);
    }
  }

  return 0;
}
