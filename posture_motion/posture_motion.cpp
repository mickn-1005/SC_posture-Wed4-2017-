// simulator of S/C's posture controll

#include <iostream>
#include <cassert>
#include <cmath>

#include "posture.h"
#include "posture.cpp"

int main(void) {
  std::vector<float> q,p;
  p = {0,0,0};
  q = q0;
  quaternion quat;

  vecs vecs;
  vecs.disp(&p);

  quat.setquat(&q);
  quat.showquat();
  p = quat.getquat();

  std::cout << "----" << '\n';

  vecs.disp(&p);

  return 0;
}
