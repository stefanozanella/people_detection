#include "window_selector.h"

#include <algorithm>
#include <boost/function.hpp>
#include <pcl/common/io.h>

using std::max;
using std::min;
using pcl::copyPointCloud;

WindowSelector::WindowSelector() :
  input (new PointCloudT),
  viewer ("window selector"),
  window_extractor (false),
  window (new PointCloudT)
{
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  int left, right;
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right);

  viewer.addPointCloud<PointT>(
      input,
      RGBHandler(input),
      "input",
      left);

  viewer.addPointCloud<PointT>(
      window,
      RGBHandler(window),
      "window",
      right);

  viewer.registerPointPickingCallback(&WindowSelector::relocateWindow, *this);
  viewer.registerKeyboardCallback(&WindowSelector::keyboardInteraction, *this);
}

void WindowSelector::setInputCloud(const PointCloudT::Ptr _input) {
  input = _input;

  search.setInputCloud(input);
  window_extractor.setInputCloud(input);

  viewer.updatePointCloud<PointT>(
      input,
      RGBHandler(input),
      "input");
}

WindowSelector::~WindowSelector() {
}

void WindowSelector::relocateWindow(const PointPickingEvent &event, void*) {
  PointT picked;
  event.getPoint(picked.x, picked.y, picked.z);

  std::vector<int> indices (1);
  std::vector<float> distances (1);
  search.nearestKSearch(picked, 1, indices, distances);

  window_y = min(indices[0] / input->width, input->height - win_size);
  window_x = min(indices[0] % input->width, input->width - win_size);

  updateWindow();
}

void WindowSelector::keyboardInteraction(const KeyboardEvent &event, void*) {
  if (event.keyDown()) {
    if (event.getKeySym() == "Up" ||
        event.getKeySym() == "Down" ||
        event.getKeySym() == "Left" ||
        event.getKeySym() == "Right") {
      shiftWindow(event.getKeySym());
    }

    if (event.getKeySym() == "s") {
      saveCurrentWindow();
    }
  }
}

void WindowSelector::shiftWindow(const string &direction) {
  if (direction == "Up")
    window_y = max(window_y - 1, uint32_t(0));
  else if (direction == "Down")
    window_y = min(window_y + 1, input->height - win_size);
  else if (direction == "Left")
    window_x = max(window_x - 1, uint32_t(0));
  else if (direction == "Right")
    window_x = min(window_x + 1, input->width - win_size);

  updateWindow();
}

void WindowSelector::updateWindow() {
  window_extractor.setIndices(window_y, window_x, win_size, win_size);
  window_extractor.filter(*window);

  viewer.updatePointCloud<PointT>(
      window,
      RGBHandler(window),
      "window");
}

void WindowSelector::saveCurrentWindow() {
  PointCloudT::Ptr face (new PointCloudT);
  copyPointCloud(*window, *face);

  _faces.push_back(face);
}

void WindowSelector::spin() {
  viewer.spin();
}

vector<PointCloudT::Ptr> WindowSelector::faces() {
  return _faces;
}
