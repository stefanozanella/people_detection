#include "window_selector.h"
#include <boost/function.hpp>

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
}

WindowSelector::~WindowSelector() {
}

void WindowSelector::relocateWindow(const PointPickingEvent &event, void*) {
  PointT picked;
  event.getPoint(picked.x, picked.y, picked.z);

  std::vector<int> indices (1);
  std::vector<float> distances (1);
  search.nearestKSearch(picked, 1, indices, distances);

  int x = indices[0] / input->width, y = indices[0] % input -> width;

  window_extractor.setIndices(x, y, win_size, win_size);
  window_extractor.filter(*window);

  viewer.updatePointCloud<PointT>(
      window,
      RGBHandler(window),
      "window");
}

void WindowSelector::spin() {
  viewer.spin();
}
