#ifndef RECT_H
#define RECT_H

#include <iostream>

struct Rect {
  int x, y, width, height, multiplier;

  Rect(int x, int y, int width, int height, int multiplier);
  friend std::ostream& operator<<(std::ostream& os, const Rect& rectangle);
};

#endif
