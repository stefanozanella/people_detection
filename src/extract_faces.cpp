#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "window_selector.h"

using pcl::io::loadPCDFile;
using pcl::io::savePCDFileASCII;
using std::ostringstream;

int main(int argc, char** argv) {
  PointCloudT::Ptr cloud (new PointCloudT);

  if (loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    cout << "Couldn't load cloud file " << argv[1] << endl;
    return -1;
  }

  WindowSelector selector (cloud);
  selector.spin();

  ExtractIndices face_extractor (false);
  face_extractor.setInputCloud(cloud);

  for (int k = 0; k < selector.faces().size(); k++) {
    PointCloudWindow faceCoords = selector.faces().at(k);

    PointCloudT::Ptr face (new PointCloudT);

    face_extractor.setIndices(faceCoords.y, faceCoords.x, faceCoords.size, faceCoords.size);
    face_extractor.filter(*face);

    ostringstream filename;
    filename << "dataset/positive/" << k << ".pcd";
    savePCDFileASCII(filename.str(), *face);
  }

  return 0;
}
