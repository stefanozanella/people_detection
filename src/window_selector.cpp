#include "window_selector.h"

#include <algorithm>
#include <boost/function.hpp>
#include <pcl/common/io.h>
#include <cmath>

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

/**
 * The picked point is going to be the center of the new window. For this
 * reason, we need to ensure we don't go over the upper and lower boundaries of
 * the images. This means we need to:
 * - calculate the index in the point's array for the point that has been
 *   selected
 * - from this index, calculate the equivalent row and column number (window_y
 *   and window_x respectively):
 *     y = index / input->width
 *     x = index % input->width
 * - given the pair of coordinates above, ensure we can span half the window in
 *   both dimensions:
 *     y = min(y, input->width - win_size / 2)
 *     x = min(x, input->height - win_size / 2)
 * - these new coordinates need to be translated of half a window up and to the
 *   left, to get coordinates of the top-left corner of the window:
 *     y = y - win_size / 2
 *     x = x - win_size / 2
 * - at this point we need to ensure that by shifting the window we didn't go
 *   out of bounds on the other side (i.e. both coordinates are > 0):
 *     y = max(0, y)
 *     x = max(0, x)
 *
 * In addition to this, here we dynamically calculate the preferred window size
 * as the ratio between the base window size and the selected point's distance.
 */
void WindowSelector::relocateWindow(const PointPickingEvent &event, void*) {
  PointT picked;
  event.getPoint(picked.x, picked.y, picked.z);

  std::vector<int> indices (1);
  std::vector<float> distances (1);
  search.nearestKSearch(picked, 1, indices, distances);

  win_size = ceil(base_win_size / input->at(indices[0]).z);

  window_y = max(0, (int) min(indices[0] / input->width, input->height - win_size / 2) - win_size / 2);
  window_x = max(0, (int) min(indices[0] % input->width, input->width - win_size / 2) - win_size / 2);

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
    window_y = max((int)(window_y - 1), 0);
  else if (direction == "Down")
    window_y = min(window_y + 1, input->height - win_size);
  else if (direction == "Left")
    window_x = max(int(window_x - 1), 0);
  else if (direction == "Right")
    window_x = min(window_x + 1, input->width - win_size);

  updateWindow();
}

void WindowSelector::updateWindow() {
  window_extractor.setIndices(window_y, window_x, win_size, win_size);
  window_extractor.filter(*window);
  window->width = window->height = win_size;

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
