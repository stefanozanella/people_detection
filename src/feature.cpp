#include <algorithm>
#include "feature.h"

Feature::Feature(uint32_t base_size) :
  base_size (base_size)
{}

Feature& Feature::operator<<(const Rect &rectangle) {
  rectangles.push_back(rectangle);

  return *this;
}

float Feature::apply(const TrainingSample &sample) const {
  float feature_value = 0;
  for (
    vector<Rect>::const_iterator rectangle = rectangles.begin();
    rectangle != rectangles.end();
    rectangle++
  ) {
    feature_value +=
      rectangle->multiplier *
      sample.scaled_integral_sum(
        rectangle->x,
        rectangle->y,
        rectangle->x + rectangle->width - 1,
        rectangle->y + rectangle->height - 1,
        base_size
      );
  }

  return feature_value;
}

std::ostream& operator<<(std::ostream& os, const Feature& feature) {
  std::copy(
    feature.rectangles.begin(),
    feature.rectangles.end(),
    std::ostream_iterator<Rect>(os, "\n")
  );

  return os;
}
