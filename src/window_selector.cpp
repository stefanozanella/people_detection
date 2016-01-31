#include "window_selector.h"

#include <algorithm>
#include <boost/function.hpp>

using std::max;
using std::min;

PointCloudWindow::PointCloudWindow(
    uint32_t x,
    uint32_t y,
    uint32_t size) :
  x (x),
  y (y),
  size (size)
{}

WindowSelector::WindowSelector(const PointCloudT::Ptr input) :
  input (input),
  viewer ("window selector"),
  window_extractor (false),
  window (new PointCloudT)
{
  search.setInputCloud(input);
  window_extractor.setInputCloud(input);

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

WindowSelector::~WindowSelector() {
}

void WindowSelector::relocateWindow(const PointPickingEvent &event, void*) {
  PointT picked;
  event.getPoint(picked.x, picked.y, picked.z);

  std::vector<int> indices (1);
  std::vector<float> distances (1);
  search.nearestKSearch(picked, 1, indices, distances);

  window_y = indices[0] / input->width;
  window_x = indices[0] % input -> width;

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
    window_y = min(window_y + 1, input->height);
  else if (direction == "Left")
    window_x = max(window_x - 1, uint32_t(0));
  else if (direction == "Right")
    window_x = min(window_x + 1, input->width);

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
  _faces.push_back(PointCloudWindow(window_x, window_y, win_size));
}

void WindowSelector::spin() {
  viewer.spin();
}

vector<PointCloudWindow> WindowSelector::faces() {
  return _faces;
}
