#ifndef RECT_H
#define RECT_H

#include <iostream>
#include "storage.h"

struct Rect {
  int x, y, width, height, multiplier;

  Rect(int x, int y, int width, int height, int multiplier);
  void save(Storage& storage) const;
  friend std::ostream& operator<<(std::ostream& os, const Rect& rectangle);
};

#endif
