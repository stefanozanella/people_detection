#include "rect.h"

Rect::Rect(int x, int y, int width, int height, int multiplier) :
  x (x),
  y (y),
  width (width),
  height (height),
  multiplier (multiplier)
{}

std::ostream& operator<<(std::ostream& os, const Rect& rectangle) {
  return os <<
      rectangle.multiplier << " ~ " <<
      "(" <<
      rectangle.width << "x" << rectangle.height <<
      "+" << rectangle.x << "+" << rectangle.y <<
      ")";
}
